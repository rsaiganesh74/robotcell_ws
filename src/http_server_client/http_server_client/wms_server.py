#!/usr/bin/env python3

from flask import Flask, request, jsonify
import logging
import threading

app = Flask(__name__)
log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)

class WMSState:
    def __init__(self):
        self.lock = threading.Lock()
        self.reset_requested = True
        self.response_data_reset = {
            "pickId": "Waiting for a Request",
            "pickSuccessful": "Waiting for a Request",
            "errorMessage": "Waiting for a Request",
            "itemBarcode": "Waiting for a Request"
        }
        self.response_data = self.response_data_reset.copy()
    
    def update_response(self, data):
        with self.lock:
            self.response_data = data
            self.reset_requested = False
    
    def reset(self):
        with self.lock:
            self.reset_requested = True
            self.response_data = self.response_data_reset.copy()
    
    def get_response(self):
        with self.lock:
            if self.reset_requested:
                return self.response_data_reset
            else:
                return self.response_data

wms_state = WMSState()

@app.route('/confirmpick', methods=['POST'])
def confirm_pick():
    data = request.get_json()
    if not data:
        return jsonify({"error": "Invalid JSON data"}), 400
    wms_state.update_response(data)
    app.logger.info(f"Confirmation received: {data}")
    return jsonify({"status": "WMS received confirmation"}), 200

@app.route('/response_reset', methods=['POST'])
def response_reset():
    wms_state.reset()
    return jsonify({"status": "Reset request acknowledged"}), 200

@app.route('/response_status', methods=['GET'])
def response_status():
    return jsonify(wms_state.get_response())

if __name__ == "__main__":
    app.run(port=8081, debug=True)
