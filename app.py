import json
import requests
import base64
from flask import Flask, Response, render_template, jsonify

app = Flask(__name__)

GATEWAY_URL = "http://localhost:8000"  # Adjust to your Go gateway host/port

@app.route('/image')
def get_image():
    try:
        res = requests.get(f"{GATEWAY_URL}/image")
        res.raise_for_status()
        image_json = res.json()
        image_data = image_json['image_base64']
        image_bytes = base64.b64decode(image_data)
        mimetype = f"image/{image_json['format']}"  # e.g. image/jpeg
        return Response(image_bytes, mimetype=mimetype)
    except requests.RequestException as e:
        return f"Failed to get image: {e}", 500

@app.route('/trailer')
def get_trailer():
    try:
        res = requests.get(f"{GATEWAY_URL}/trailer")
        res.raise_for_status()
        return jsonify(res.json())
    except requests.RequestException as e:
        return f"Failed to get trailer: {e}", 500

@app.route('/velocity')
def get_velocity():
    try:
        res = requests.get(f"{GATEWAY_URL}/velocity")
        res.raise_for_status()
        return jsonify(res.json())
    except requests.RequestException as e:
        return f"Failed to get velocity: {e}", 500


@app.route('/')
def index():
    return render_template('result.html')

if __name__ == '__main__':
    app.run(host="0.0.0.0", port=8080, debug=True)