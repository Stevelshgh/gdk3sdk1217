all: abc.c 
	@echo hello make
	gcc -o gemake $^

abc.c:
	@echo "#include <stdio.h>\n int main(){puts("hello!\n");}" > $@

gedu: 
	echo "let's ge software together!"

.PHONY: clean
clean:
	rm abc.c gemake -rf

test:
	echo "begin testing:"
	gemake 