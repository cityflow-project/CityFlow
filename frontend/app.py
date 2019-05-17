from flask import Flask, request, send_from_directory, jsonify, render_template
# set the project root directory as the static folder, you can set others.
app = Flask(__name__, static_url_path='', static_folder='')

@app.route('/')
def index():
    data = {
        "roadnetFile": "replay/" + request.args.get('roadnetFile'),
        "logFile": "replay/" + request.args.get('logFile')
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