// X-Y plot
class PlotXY {
  constructor(div_id) {
    // Plot settings
    this.data =
        [ {x : [], y : [], mode : 'lines', line : {color : '#80CAF6'}} ];
    this.layout = {
      autosize : false,
      width : 300,
      height : 300,
      margin : {l : 30, r : 30, b : 30, t : 30, pad : 0},
    };
    this.buf_size = 100;

    // Create plot
    this.div_id = div_id;
    this.div = document.getElementById(this.div_id);
    if (this.div == null) {
      this.div = document.createElement('div');
      this.div.setAttribute('id', this.div_id);
      this.div.setAttribute('class', 'viz_plot');
      document.body.appendChild(this.div);
      Plotly.newPlot(this.div_id, this.data, this.layout, {});
    }
  }

  reset() { Plotly.newPlot(this.div_id, this.data, this.layout, {}); }

  update(x_new, y_new) {
    Plotly.extendTraces(this.div_id,
                        {
                          x : [ [ x_new ] ],
                          y : [ [ y_new ] ],
                        },
                        [ 0 ],
                        this.buf_size);
  }
}

function setup_session() {
  // Plot xy
  var xy_plot = new PlotXY("test");

  // Create websocket connection
  var ws = new WebSocket("ws://0.0.0.0:8080");

  // On open
  ws.onopen = function(event) {
    console.log("Connected to server!");
    xy_plot.reset();
  };

  // On message
  ws.onmessage = function(event) {
    var data = JSON.parse(event.data);
    xy_plot.update(data['index'], data['val']);
  };

  // On close
  ws.onclose = function() {
    // Retry to connect to server every 1s
    setTimeout(function() { setup_session(); }, 500);
  };
}
