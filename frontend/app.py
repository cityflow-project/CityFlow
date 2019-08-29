from flask import Flask, request, send_from_directory, jsonify, render_template
# set the project root directory as the static folder, you can set others.
app = Flask(__name__, static_url_path='', static_folder='')

@app.route('/')
def index():
    roadnet_file_path = request.args.get('roadnetFile')
    log_file_path = request.args.get('logFile')
    
    roadnet_file_root_dir = "replay" if roadnet_file_path.startswith("/") else "replay/"
    log_file_root_dir = "replay" if log_file_path.startswith("/") else "replay/"
    
    data = {
        "roadnetFile": roadnet_file_root_dir + roadnet_file_path,
        "logFile": log_file_root_dir + log_file_path
    }
    return render_template('index.html', data=data)

@app.route('/replay/<path:path>')
def replay(path):
    return send_from_directory('replay/', path)

@app.route('/library/<path:path>')
def library(path):
    return send_from_directory('library/', path)

if __name__=='__main__':
    app.run(host='0.0.0.0', port=8080)
