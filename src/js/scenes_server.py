from SimpleHTTPServer import SimpleHTTPRequestHandler
import BaseHTTPServer
import os

class CORSRequestHandler (SimpleHTTPRequestHandler):
    def end_headers (self):
        self.send_header('Access-Control-Allow-Origin', '*')
        SimpleHTTPRequestHandler.end_headers(self)
    def do_GET(self):
        pi = self.path.find('+')
        if pi != -1:
            self.path = self.path[:pi]
        return SimpleHTTPRequestHandler.do_GET(self)

if __name__ == '__main__':
    dir = os.path.dirname(os.path.abspath(__file__))
    print "in dir", dir
    os.chdir(dir)
    BaseHTTPServer.test(CORSRequestHandler, BaseHTTPServer.HTTPServer)
    

# missing 4,9,13,14,17,18