all: bmp_to_array

bmp_to_array: bmp_to_array.c libbmp/libbmp.a
	${CC} -o bmp_to_array $< libbmp/libbmp.a
