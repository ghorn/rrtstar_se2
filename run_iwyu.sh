#!/usr/bin/env bash

check_also_flags=""
for header in $(git ls-files '*.hpp')
do
    if ! test -f "${header/.hpp/.cpp}"
    then
	check_also_flags="$check_also_flags -Xiwyu --check_also=$header"
    fi
done

rm -f iwyu.out

for file in $(git ls-files '*.cpp')
do
    echo "======================================"
    echo "running iwyu on $file"
    iwyu \
	$check_also_flags \
	-Xiwyu --max_line_length=100 \
	-Xiwyu --mapping_file=iwyu_mapping.imp \
	-Xiwyu --no_fwd_decls \
	-std=c++17 \
	-I. \
	-isystem $(clang -print-resource-dir)/include \
	-isystem /usr/include/freetype2 \
	-c $file \
	2>> iwyu.out
    retval=$?
    #echo "exit code is $retval"
    if [ $retval -eq 1 ]
    then
	cat iwyu.out
	echo "---------------------------------------------"
	echo "compilation failed for $file, see error above"
	exit 1
    fi
done

fix_include --nosafe_headers --comments < iwyu.out

rm iwyu.out
