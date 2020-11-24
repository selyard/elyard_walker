cppcheck --enable=all --std=c++11 -I include/ --suppress=missingIncludeSystem $( find . -name \*.hpp -or -name \*.cpp -or -name \*.h | grep -vE -e "include/" -e ".git/" )
