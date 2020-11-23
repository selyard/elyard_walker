cpplint $( find . -name \*.hpp -or -name \*.cpp -or -name \*.h | grep -vE -e "include/" -e ".git/" )
