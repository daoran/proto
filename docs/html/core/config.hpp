<!-- CLASSES --><div class="cpp_class">
  <pre><code>struct config_t {
  std::string file_path;
  YAML::Node root;
  bool ok;

  config_t();
  config_t(const std::string &amp;file_path_);
  ~config_t();
};</code></pre>
  
</div> <!-- .cpp_class -->
<!-- FUNCTIONS -->
<h2>Functions</h2>

  <div class="cpp_func" onclick=show_doc(this)>
    <pre><code class="cpp">int yaml_load_file(const std::string file_path, YAML::Node &amp;root);
</code></pre>
    <div class="doc"><p>
Load YAML file.
</br><b>Returns</b> 0 for success or -1 for failure.


</p></div>
  </div> <!-- .cpp_func -->

  <div class="cpp_func" onclick=show_doc(this)>
    <pre><code class="cpp">int yaml_get_node(const config_t &amp;config,
                  const std::string &amp;key,
                  const bool optional,
                  YAML::Node &amp;node);
</code></pre>
    <div class="doc"><p>
Get YAML node containing the parameter value.
</br><b>Returns</b> 0 for success or -1 for failure.


</p></div>
  </div> <!-- .cpp_func -->

  <div class="cpp_func" onclick=show_doc(this)>
    <pre><code class="cpp">template &lt;typename T&gt;
size_t yaml_check_vector(const YAML::Node &amp;node,
                         const std::string &amp;key,
                         const bool optional);
</code></pre>
    <div class="doc"><p>
Check size of vector in config file and returns the size.


</p></div>
  </div> <!-- .cpp_func -->

  <div class="cpp_func" onclick=show_doc(this)>
    <pre><code class="cpp">template &lt;typename T&gt;
void yaml_check_matrix(const YAML::Node &amp;node,
                       const std::string &amp;key,
                       const bool optional,
                       size_t &amp;rows,
                       size_t &amp;cols);
</code></pre>
    <div class="doc"><p>
Check matrix to make sure that the parameter has the data field "rows",
"cols" and "data". It also checks to make sure the number of values is the
same size as the matrix.


</p></div>
  </div> <!-- .cpp_func -->

  <div class="cpp_func" onclick=show_doc(this)>
    <pre><code class="cpp">template &lt;typename T&gt;
void yaml_check_matrix(const YAML::Node &amp;node,
                       const std::string &amp;key,
                       const bool optional);
</code></pre>
    <div class="doc"><p>None</p></div>
  </div> <!-- .cpp_func -->

  <div class="cpp_func" onclick=show_doc(this)>
    <pre><code class="cpp">template &lt;typename T&gt;
void parse(const config_t &amp;config,
           const std::string &amp;key,
           T &amp;out,
           const bool optional);
</code></pre>
    <div class="doc"><p>None</p></div>
  </div> <!-- .cpp_func -->

  <div class="cpp_func" onclick=show_doc(this)>
    <pre><code class="cpp">template &lt;typename T&gt;
void parse(const config_t &amp;config,
           const std::string &amp;key,
           std::vector&lt;T&gt; &amp;out,
           const bool optional);
</code></pre>
    <div class="doc"><p>None</p></div>
  </div> <!-- .cpp_func -->

  <div class="cpp_func" onclick=show_doc(this)>
    <pre><code class="cpp">void parse(const config_t &amp;config,
           const std::string &amp;key,
           vec2_t &amp;vec,
           const bool optional);
</code></pre>
    <div class="doc"><p>None</p></div>
  </div> <!-- .cpp_func -->

  <div class="cpp_func" onclick=show_doc(this)>
    <pre><code class="cpp">void parse(const config_t &amp;config,
           const std::string &amp;key,
           vec3_t &amp;vec,
           const bool optional);
</code></pre>
    <div class="doc"><p>None</p></div>
  </div> <!-- .cpp_func -->

  <div class="cpp_func" onclick=show_doc(this)>
    <pre><code class="cpp">void parse(const config_t &amp;config,
           const std::string &amp;key,
           vec4_t &amp;vec,
           const bool optional);
</code></pre>
    <div class="doc"><p>None</p></div>
  </div> <!-- .cpp_func -->

  <div class="cpp_func" onclick=show_doc(this)>
    <pre><code class="cpp">void parse(const config_t &amp;config,
           const std::string &amp;key,
           vecx_t &amp;vec,
           const bool optional);
</code></pre>
    <div class="doc"><p>None</p></div>
  </div> <!-- .cpp_func -->

  <div class="cpp_func" onclick=show_doc(this)>
    <pre><code class="cpp">void parse(const config_t &amp;config,
           const std::string &amp;key,
           mat2_t &amp;mat,
           const bool optional);
</code></pre>
    <div class="doc"><p>None</p></div>
  </div> <!-- .cpp_func -->

  <div class="cpp_func" onclick=show_doc(this)>
    <pre><code class="cpp">void parse(const config_t &amp;config,
           const std::string &amp;key,
           mat3_t &amp;mat,
           const bool optional);
</code></pre>
    <div class="doc"><p>None</p></div>
  </div> <!-- .cpp_func -->

  <div class="cpp_func" onclick=show_doc(this)>
    <pre><code class="cpp">void parse(const config_t &amp;config,
           const std::string &amp;key,
           mat4_t &amp;mat,
           const bool optional);
</code></pre>
    <div class="doc"><p>None</p></div>
  </div> <!-- .cpp_func -->

  <div class="cpp_func" onclick=show_doc(this)>
    <pre><code class="cpp">void parse(const config_t &amp;config,
           const std::string &amp;key,
           matx_t &amp;mat,
           const bool optional);
</code></pre>
    <div class="doc"><p>None</p></div>
  </div> <!-- .cpp_func -->

  <div class="cpp_func" onclick=show_doc(this)>
    <pre><code class="cpp">void parse(const config_t &amp;config,
           const std::string &amp;key,
           cv::Mat &amp;mat,
           const bool optional);
</code></pre>
    <div class="doc"><p>None</p></div>
  </div> <!-- .cpp_func -->

</div> <!-- #content -->