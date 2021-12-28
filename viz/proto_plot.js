function assert(condition, message) {
  if (!condition) {
    throw message || "Assertion failed!";
  }
}

// X-Y plot
class PlotXY {
  constructor(plot_config) {
    assert(Array.isArray(plot_config["traces"]));
    this.title = plot_config["title"];
    this.width = plot_config["width"];
    this.height = plot_config["height"];
    this.div_id = plot_config["div_id"];
    this.buf_size = plot_config["buf_size"];
    this.traces = plot_config["traces"];
    this.xlabel = plot_config["axis_labels"]["xaxis"];
    this.ylabel = plot_config["axis_labels"]["yaxis"];
    this.showlegend = plot_config["showlegend"];

    this.traces.forEach(function(trace) {
      if (trace["x"] == undefined) {
        trace["x"] = [];
      }
      if (trace["y"] == undefined) {
        trace["y"] = [];
      }
    });

    // Prepare traces
    this.trace_names = [];
    for (var i = 0; i < this.traces.length; i++) {
      this.trace_names.push(this.traces[i].name);
    }

    // Plot settings
    this.layout = {
      autosize : false,
      width : this.width,
      height : this.height,
      margin : {l : 35, r : 35, b : 35, t : 35, pad : 0},
      title : this.title,
      xaxis : {"title" : this.xlabel},
      yaxis : {"title" : this.ylabel},
      font : {family : "Times", size : 10, color : "#000"},
      showlegend : this.showlegend,
      legend : {x : 1.0, y : 1.0, xanchor : "right"}
    };

    // Create plot
    this.div = document.getElementById(this.div_id);
    if (this.div == null) {
      this.div = document.createElement("div");
      this.div.style.width = this.layout.width;
      this.div.style.height = this.layout.height;
      this.div.setAttribute("id", this.div_id);
      this.div.setAttribute("class", "viz_plot");
      document.body.appendChild(this.div);
      Plotly.newPlot(this.div_id, this.traces, this.layout, {});
    }
  }

  reset() { Plotly.newPlot(this.div_id, this.traces, this.layout, {}); }

  update(data) {
    var data_new = {x : [], y : []};
    this.trace_names.forEach(function(trace_name) {
      data_new["x"].push([ data[trace_name]["x"] ]);
      data_new["y"].push([ data[trace_name]["y"] ]);
    });
    Plotly.extendTraces(this.div_id, data_new, this.trace_idxs, this.buf_size);
  }
}

class MultiPlot {
  constructor() {
    this.plot_width = 300;
    this.plot_height = 250;
    this.buf_size = 300;
    this.plots = {};
    this._setup_plots();
  }

  _setup_plots() {
    this._pos_xy_plot();
    this._pos_z_plot();
    // this._att_plot();
    this._pos_error_plot();
    this._att_error_plot();
    this._reproj_error_plot();
  }

  _pos_xy_plot() {
    var plot_conf = {
      title : "Position X-Y",
      width : this.plot_width,
      height : this.plot_height,
      div_id : "pos_xy_plot",
      buf_size : this.buf_size,
      traces : [
        {name : "Ground-Truth", "line" : {color : "#FF0000", width : 1}},
        {name : "Estimate", "line" : {color : "#1A5277", width : 1}}
      ],
      axis_labels : {xaxis : "x [m]", yaxis : "y [m]"},
      showlegend : true
    };
    this.plots[plot_conf.title] = new PlotXY(plot_conf);
  }

  _pos_z_plot() {
    var plot_conf = {
      title : "Position Z",
      width : this.plot_width,
      height : this.plot_height,
      div_id : "pos_z_plot",
      buf_size : this.buf_size,
      traces : [
        {name : "Ground-Truth", "line" : {color : "#FF0000", width : 1}},
        {name : "Estimate", "line" : {color : "#1A5277", width : 1}}
      ],
      axis_labels : {xaxis : "Time [s]", yaxis : "Altitude [m]"},
      showlegend : true
    };
    this.plots[plot_conf.title] = new PlotXY(plot_conf);
  }

  _pos_error_plot() {
    var plot_conf = {
      title : "Position Error",
      width : this.plot_width,
      height : this.plot_height,
      div_id : "pos_error_plot",
      buf_size : this.buf_size,
      traces : [ {name : "Error", "line" : {color : "#1A5277", width : 1}} ],
      axis_labels : {xaxis : "Time [s]", yaxis : "Position Error [m]"},
      showlegend : false
    };
    this.plots[plot_conf.title] = new PlotXY(plot_conf);
  }

  _att_plot() {
    var plot_conf = {
      title : "Attitude",
      width : this.plot_width,
      height : this.plot_height,
      div_id : "att_plot",
      buf_size : this.buf_size,
      traces : [
        {name : "Roll", "line" : {color : "#FF0000", width : 1}},
        {name : "Pitch", "line" : {color : "#1A5277", width : 1}},
        {name : "Yaw", "line" : {color : "#1A5277", width : 1}}
      ],
      axis_labels : {xaxis : "Time [s]", yaxis : "Attitude [deg]"},
      showlegend : true
    };
    this.plots[plot_conf.title] = new PlotXY(plot_conf);
  }

  _att_error_plot() {
    var plot_conf = {
      title : "Attitude Error",
      width : this.plot_width,
      height : this.plot_height,
      div_id : "att_error_plot",
      buf_size : this.buf_size,
      traces : [ {name : "Error", "line" : {color : "#1A5277", width : 1}} ],
      axis_labels : {xaxis : "Time [s]", yaxis : "Angle Error [deg]"},
      showlegend : false
    };
    this.plots[plot_conf.title] = new PlotXY(plot_conf);
  }

  _reproj_error_plot() {
    var plot_conf = {
      title : "Reprojection Error",
      width : this.plot_width,
      height : this.plot_height,
      div_id : "reproj_error_plot",
      buf_size : this.buf_size,
      traces : [
        {name : "Mean", "line" : {color : "#FF0000", width : 1}},
        {name : "RMSE", "line" : {color : "#FF0000", width : 1}},
      ],
      axis_labels : {xaxis : "Time [s]", yaxis : "Reprojection Error [px]"},
      showlegend : false
    };
    this.plots[plot_conf.title] = new PlotXY(plot_conf);
  }

  reset() {
    for (var plot_name in this.plots) {
      this.plots[plot_name].reset();
    }
  }

  update(data) {
    for (var plot_name in this.plots) {
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
  ws.onopen = function(event) {
    console.log("Connected to server!");
    multi_plot.reset();
  };

  // On message
  ws.onmessage = function(event) {
    var data = JSON.parse(event.data);
    console.log(data);
    multi_plot.update(data);
  };

  // On close
  ws.onclose = function() {
    // Retry to connect to server every 1s
    setTimeout(function() { setup_session(); }, 1000);
  };
}
