from SimpleHTTPServer import SimpleHTTPRequestHandler
import BaseHTTPServer
import os

class CORSRequestHandler (SimpleHTTPRequestHandler):
    def end_headers (self):
        self.send_header('Access-Control-Allow-Origin', '*')
        SimpleHTTPRequestHandler.end_headers(self)

if __name__ == '__main__':
    dir = os.path.dirname(os.path.abspath(__file__))
    print "in dir", dir
    os.chdir(dir)
    BaseHTTPServer.test(CORSRequestHandler, BaseHTTPServer.HTTPServer)