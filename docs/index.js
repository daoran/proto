// Render latex
var katex_options = {
  throwOnError: true,
  delimiters: [
    {left: "$$", right: "$$", display: true},
    {left: "$", right: "$", display: false}
  ],
  macros: {
    // General
    "\\Vec": "\\mathbf{#1}",
    "\\Mat": "\\mathbf{#1}",
    "\\real": "\\rm I\\!R",
    "\\ones": "{\\Vec{1}}",
    "\\Ones": "{\\Vec{1}_{#1\\times#2}}",
    "\\zeros": "{\\Vec{0}}",
    "\\Zeros": "{\\Vec{0}_{#1\\times#2}}",
    "\\Norm": "{\\|#1\\|}",
    "\\LargeNorm": "{\\left\\lVert#1\\right\\rVert}",
    "\\I": "{\\Mat{I}}",
    "\\Skew": "{\\lfloor #1 \\enspace \\times \\rfloor}",
    "\\Argmin": "\\underset{#1}{\\text{argmin }}",
    "\\transpose": "T",
    "\\Transpose": "{#1^{\\transpose}}",
    "\\Inv": "{#1^{-1}}",
    "\\LargeNorm": "\\left\\lVert#1\\right\\rVert}",
    "\\Trace": "\\text{tr}(#1)",
    "\\Rank": "\\text{rank}(#1)}",
    "\\Bigslant": "{\\left/\\right}",
    "\\cost": "J",
    "\\argmin": "\\mathop{\\mathrm{argmin}",
    // Transforms
    "\\frame": "{\\mathcal{F}}",
    "\\rot": "{\\Mat{C}}",
    "\\trans": "{\\Vec{r}}",
    "\\quat": "{\\Vec{q}}",
    "\\tf": "{\\Mat{T}}",
    // Standard terms
    "\\state": "{\\Vec{x}}",
    "\\pos": "{\\Vec{r}}",
    "\\vel": "{\\Vec{v}}",
    "\\acc": "{\\Vec{a}}",
    "\\angvel": "{\\boldsymbol{\\omega}}",
    "\\gravity": "{\\Vec{g}}",
    "\\noise": "{\\Vec{n}}",
    "\\bias": "{\\Vec{b}}",
    // Gyroscope
    "\\gyr": "{\\angvel}",
    "\\gyrMeas": "{\\angvel_{m}}",
    "\\gyrNoise": "{\\noise_{g}}",
    "\\gyrBias": "{\\bias_{g}}",
    // Accelerometer
    "\\accMeas": "{\\acc_{m}}",
    "\\accNoise": "{\\noise_{a}}",
    "\\accBias": "{\\bias_{a}}"
  }
};

function http_get(url) {
  var xmlHttp = new XMLHttpRequest();
  xmlHttp.open( "GET", url, false ); // false for synchronous request
  xmlHttp.send( null );
  return xmlHttp.responseText;
}

function http_get_async(url, callback) {
  var xmlHttp = new XMLHttpRequest();
  xmlHttp.onreadystatechange = function() {
    if (xmlHttp.readyState == 4 && xmlHttp.status == 200)
      callback(xmlHttp.responseText);
  }
  xmlHttp.open("GET", url, true); // true for asynchronous
  xmlHttp.send(null);
}

function render_note() {
  var key = location.hash.replace("#", "");
  var file = key.replace("-", "/") + ".html";
  var html = http_get(file);
  document.getElementById("readme").innerHTML = "";
  document.getElementById("notes").innerHTML = html;

  // Render maths
  renderMathInElement(document.body, katex_options);

  // Enumerate equations
  var equations = document.getElementsByTagName("equation");
  for (var i = 0; i < equations.length; i++) {
    var eq = equations[i];
    var eq_num = (i + 1);
    var html = "<div style='text&#45;align:center;margin&#45;top:20px;margin&#45;bottom:20px'>";
    html += katex.renderToString("\\begin{aligned}" + eq.textContent + "\\end{aligned}", katex_options);
    html += "</div>";
    eq.innerHTML = html;
  }
}

document.addEventListener("DOMContentLoaded", function() {
  if (location.hash == "") {
    // Load README
    var readme_url = "https://raw.githubusercontent.com/chutsu/proto/master/README.md";
    http_get_async(readme_url, function(data) {
      var readme_section = document.getElementById("readme");
      var md = new showdown.Converter();
      readme_section.innerHTML = md.makeHtml(data);
    });
  } else {
    render_note();
  }
});

window.addEventListener("hashchange", function(event) {
  render_note();
});
