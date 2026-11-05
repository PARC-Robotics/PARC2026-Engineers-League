"""Dashboard web server for the referee (Python standard library only).

GET  /                       projector view
GET  /judge?token=...        judge view (same page with controls)
GET  /api/state              current state as JSON
GET  /api/events             state as a Server-Sent Events stream
GET  /api/image/<a>/<i>      image of visit i in attempt a
POST /api/judge/<action>     judge actions; header X-Judge-Token required
"""

import json
import os
import re
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

from .run_state import RunState


def make_server(state: RunState, web_dir, host, port, token, on_change):
    with open(os.path.join(web_dir, 'index.html'), 'rb') as f:
        page = f.read()

    actions = {
        'arm': lambda b: {'order': state.arm(
            b.get('team', ''),
            RunState.parse_order(b['order']) if b.get('order') else None)},
        'restart': lambda b: state.restart(),
        'end': lambda b: state.end_run(),
        'terminate': lambda b: state.terminate(b.get('reason', '')),
        'reset': lambda b: state.reset(),
        'verdict': lambda b: state.set_verdict(int(b['index']), b.get('verdict')),
        'contact': lambda b: state.add_contact(b['kind'], 1 if int(b.get('delta', 1)) > 0 else -1),
        'voice_bonus': lambda b: state.set_voice_bonus(b.get('value')),
    }

    class Handler(BaseHTTPRequestHandler):
        protocol_version = 'HTTP/1.1'

        def log_message(self, *args):
            pass

        def _send(self, code, body, ctype='application/json'):
            if isinstance(body, (dict, list)):
                body = json.dumps(body).encode()
            elif isinstance(body, str):
                body = body.encode()
            self.send_response(code)
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            path = urlparse(self.path).path
            if path in ('/', '/judge'):
                return self._send(200, page, 'text/html; charset=utf-8')
            if path == '/api/state':
                return self._send(200, state.to_dict())
            if path == '/api/events':
                return self._stream()
            m = re.fullmatch(r'/api/image/(\d+)/(\d+)', path)
            if m:
                p = state.image_path(int(m.group(1)), int(m.group(2)))
                if p and os.path.exists(p):
                    with open(p, 'rb') as f:
                        data = f.read()
                    ctype = 'image/png' if data[:4] == b'\x89PNG' else 'image/jpeg'
                    return self._send(200, data, ctype)
            self._send(404, {'error': 'not found'})

        def _stream(self):
            self.send_response(200)
            self.send_header('Content-Type', 'text/event-stream')
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Connection', 'keep-alive')
            self.end_headers()
            try:
                while True:
                    data = json.dumps(state.to_dict())
                    self.wfile.write(f'data: {data}\n\n'.encode())
                    self.wfile.flush()
                    with state.changed:
                        state.changed.wait(timeout=0.5)
            except (BrokenPipeError, ConnectionResetError, OSError):
                pass

        def do_POST(self):
            m = re.fullmatch(r'/api/judge/(\w+)', urlparse(self.path).path)
            if not m or m.group(1) not in actions:
                return self._send(404, {'error': 'unknown action'})
            if self.headers.get('X-Judge-Token', '') != token:
                return self._send(403, {'error': 'wrong judge token'})
            length = int(self.headers.get('Content-Length', 0) or 0)
            try:
                body = json.loads(self.rfile.read(length) or b'{}')
                result = actions[m.group(1)](body) or {}
            except (ValueError, KeyError, TypeError) as e:
                return self._send(400, {'error': str(e)})
            on_change()
            self._send(200, {'ok': True, **result})

    server = ThreadingHTTPServer((host, port), Handler)
    server.daemon_threads = True
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    return server
