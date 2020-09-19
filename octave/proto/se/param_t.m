classdef param_t
  properties
    type_info = "";
    ts = 0;
    data = [];
    min_size = 0;
  endproperties

  methods
    function param = param_t(type_info, ts, data, min_size)
      assert(columns(data) == 1);
      assert(rows(data) > 1);

      param.type_info = type_info;
      param.ts = ts;
      param.data = data;
      param.min_size = min_size;
    endfunction

    function print(self)
      printf("ts: %ld\n", self.ts);

      printf("data: [");
      for i = 1:length(self.data)
        printf("%f", self.data(i));
        if (i) != length(self.data)
          printf(", ");
        endif
      endfor
      printf("]\n");

      printf("type_info: %s\n", self.type_info);
      printf("min_size: %d\n", self.min_size);
    endfunction
  endmethods
endclassdef
