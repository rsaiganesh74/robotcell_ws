from flask import Flask,request,jsonify

app = Flask(__name__)

@app.route('/confirmpick', methods=['POST'])
def confirm_pcik():
    data = request.get_json()
    print(f"Received confirmation: {data}")

    return jsonify({"status":"WMS received confirmation"}),200


if  __name__=="__main__":
    app.run(port=8081,debug=True)