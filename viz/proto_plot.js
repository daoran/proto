function assert(condition, message) {
  if (!condition) {
    throw message || "Assertion failed!";
  }
}

class PlotXY {
  constructor(plot_config) {
    // Plot config
    this.title = plot_config["title"];
    this.width = plot_config["width"];
    this.height = plot_config["height"];
    this.div_id = plot_config["div_id"];
    this.buf_size = plot_config["buf_size"];
    this.traces = plot_config["traces"];
    this.xlabel = plot_config["xlabel"];
    this.ylabel = plot_config["ylabel"];
    this.show_legend = plot_config["show_legend"];

    // Prepare traces
    assert(Array.isArray(plot_config["traces"]));
    this.trace_names = [];
    this.trace_idxs = [];
    for (var i = 0; i < this.traces.length; i++) {
      this.trace_names.push(this.traces[i]["name"]);
      this.trace_idxs.push(i);
    }

    // Plotly settings
    this.layout = {
      autosize : false,
      width : this.width,
      height : this.height,
      margin : {l : 35, r : 35, b : 35, t : 35, pad : 0},
      title : this.title,
      xaxis : {"title" : this.xlabel},
      yaxis : {"title" : this.ylabel},
      font : {family : "Times", size : 10, color : "#000"},
      showlegend : this.show_legend,
      legend : {orientation : "h", x : 0.05, y : 1.05}
    };

    // Create plot
    // -- Remove element if it already exists
    if (document.getElementById(this.div_id)) {
      document.getElementById(this.div_id).outerHTML = "";
    }
    // -- Add new element to document
    this.div = document.createElement("div");
    this.div.style.width = this.layout.width;
    this.div.style.height = this.layout.height;
    this.div.setAttribute("id", this.div_id);
    this.div.setAttribute("class", "viz_plot");
    document.body.appendChild(this.div);
    // -- Add plot to div
    Plotly.newPlot(this.div_id, this.traces, this.layout, {});
  }

  update(data) {
    // Process data
    var data_new = {x : [], y : []};
    for (var i = 0; i < this.trace_names.length; i++) {
      // Check to see if the trace name exists in the data
      const trace_name = this.trace_names[i];
      if (!(trace_name in data)) {
        const err_msg = "Error in [%s] plot! Trace_name [%s] not in data:[%s]";
        console.log(err_msg, this.title, trace_name, JSON.stringify(data));
        return false;
      }

      // Add new data to array
      data_new["x"].push([ data[trace_name]["x"] ]);
      data_new["y"].push([ data[trace_name]["y"] ]);
    }

    // Update plot
    Plotly.extendTraces(this.div_id, data_new, this.trace_idxs, this.buf_size);

    return true;
  }
}

class MultiPlot {
  constructor() {
    this.init = false;
    this.plot_width = 300;
    this.plot_height = 250;
    this.buf_size = 100;
    this.plots = {};
  }

  setup_plots(data) {
    for (var i = 0; i < data.length; i++) {
      var plot_conf = data[i];
      this.plots[plot_conf.title] = new PlotXY(plot_conf);
    }
    this.init = true;
  }

  update(data) {
    for (var plot_name in data) {
      this.plots[plot_name].update(data[plot_name]);
    }
  }
}

function setup_session() {
  // Setup
  var multi_plot = new MultiPlot();

  // Create websocket connection
  var ws = new WebSocket("ws://127.0.0.1:5000");

  // On open
  ws.onopen = function(event) { console.log("Connected to server!"); };

  // On message
  ws.onmessage = function(event) {
    var data = JSON.parse(event.data);
    if (multi_plot.init == false) {
      multi_plot.setup_plots(data);
    } else {
      multi_plot.update(data);
    }
  };

  // On close
  ws.onclose = function() {
    // Retry to connect to server every 1s
    // setTimeout(function() { setup_session(); }, 1000);
  };
}
