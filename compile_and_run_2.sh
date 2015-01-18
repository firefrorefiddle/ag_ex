pwd=`pwd`

osascript -e "tell application \"Terminal\"" \
    -e "tell application \"System Events\" to keystroke \"t\" using {command down}" \
    -e "do script \"cd $pwd; clear\" in front window" \
    -e "do script \"exec ./compile_and_run.sh -f instances/$1.prob -m scf\" in front window" \
    -e "end tell"
    > /dev/null
    
osascript -e "tell application \"Terminal\"" \
    -e "tell application \"System Events\" to keystroke \"t\" using {command down}" \
    -e "do script \"cd $pwd; clear\" in front window" \
    -e "do script \"exec ./compile_and_run.sh -f instances/$1.prob -m mcf\" in front window" \
    -e "end tell"
    > /dev/null
    
osascript -e "tell application \"Terminal\"" \
    -e "tell application \"System Events\" to keystroke \"t\" using {command down}" \
    -e "do script \"cd $pwd; clear\" in front window" \
    -e "do script \"exec ./compile_and_run.sh -f instances/$1.prob -m mtz\" in front window" \
    -e "end tell"
    > /dev/null