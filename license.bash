echo $1
sed -i -e '1 { r license-txt' -e 'N; }' $1;	
