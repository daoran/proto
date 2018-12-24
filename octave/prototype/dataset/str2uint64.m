function [ts_int] = str2uint64(ts_str)
  assert(typeinfo(ts_str) == "sq_string");

  ts_int = uint64(0);
  for i = 0:(length(ts_str) - 1)
    ts_int += str2num(ts_str(end - i)) * 10**i;
  end
endfunction
