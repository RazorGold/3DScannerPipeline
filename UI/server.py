from flask import Flask, send_from_directory, request, jsonify
import os

app = Flask(__name__, static_folder='static')
STATUS_DIR = 'status_files'

@app.route('/')
def index():
    return send_from_directory('static', 'index.html')

@app.route('/checkFile')
def check_file():
    filename = request.args.get('name')
    filepath = os.path.join(STATUS_DIR, filename)
    return jsonify({"exists": os.path.exists(filepath)})

@app.route('/<path:path>')
def serve_static(path):
    return send_from_directory('static', path)

if __name__ == '__main__':
    app.run(debug=True)
