.PHONY: lib python

lib:
	@make -C lib -s

python:
	@make -C lib -s run_tests
	@cd python && python3 proto.py

clean:
	@make -C lib -s clean
