
var running = false;

var interval;
var ws;

var CMD = {

    getData: function () {
        var msg = { "cmd": 3 };
        ws.send(JSON.stringify(msg));
    },

    run: function () {
        var msg = { "cmd": 1 };
        ws.send(JSON.stringify(msg));
        // console.log("sending " + msg);
        document.getElementById("run").textContent = "stop";

        interval = setInterval(CMD.getData, 1000 / 10);
        running = true;
    },

    stop: function () {
        clearInterval(interval);
        running = false;

        var msg = { "cmd": 2 };
        ws.send(JSON.stringify(msg));
        // console.log("sending " + msg);
        document.getElementById("run").textContent = "start";
    },
    log: function(tag){
        var msg = {"cmd": 4, "pld": tag};
        ws.send(JSON.stringify(msg));
    },

    stoplog: function(){
        var msg = {"cmd": 5};
        ws.send(JSON.stringify(msg));
    }
};



function log(tag) {

}
function stoplog() {

}
function connect() {
    if ("WebSocket" in window) {
        // alert("WebSocket is supported by your Browser!");
        // Let us open a web socket
        ws = new WebSocket("ws://localhost:4114");

        ws.onopen = function () {
            // Web Socket is connected, send data using send()
            document.getElementById("status").textContent = "CONNECTED";
            // ws.send("Message to send");
            // alert("Message is sent...");

            document.getElementById("run").addEventListener("click", function (e) {
                if (!running) {
                    CMD.run();
                } else {
                    CMD.stop();
                }
            });

        };
        ws.onmessage = function (evt) {
            var received_msg = evt.data;
            // console.log("Message is received...");
            // console.log(received_msg);
            var data = JSON.parse(received_msg);
            // console.log(data);
            if (data.cmd === 3) {
                // console.log("Received points data");
                dists = data.dists;
                angles = data.angles;
                qs = data.qs;
                // updatedData = window.performance.now();
                // console.log(dists);
            }
        };

        ws.onclose = function () {
            // websocket is closed.
            document.getElementById("status").textContent = "DISCONNECTED";
        };

        ws.onerror = function (error) {
            document.getElementById("status").textContent = 'Error detected: ' + JSON.stringify(error);
        }

    }
    else {
        // The browser doesn't support WebSocket
        alert("WebSocket NOT supported by your Browser!");
    }
}

connect();