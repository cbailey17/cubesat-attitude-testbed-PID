function Out=int2three(T1)
T1s=int2str(T1);
if size(T1s,2)==2
    T1s=['0',T1s];
elseif size(T1s,2)==1
    T1s=['00',T1s];
else
    T1s=T1s;
end
Out=T1s;
end