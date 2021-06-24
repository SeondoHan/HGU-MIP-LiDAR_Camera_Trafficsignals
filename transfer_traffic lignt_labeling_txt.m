clear all; close all; clc;


for k = 17:928
    file_name = sprintf('fc2_save_2017-01-10-144215-%04d.txt',k);
    readfile = fopen(file_name,'r');
    formatSpec = '%f';
    A=fscanf(readfile, formatSpec);
    L=size(A,1);
    temp=A;
    
    for i = 1:5:L;
        temp(i) = 0;
        temp(i+1) = ((A(3) + A(1)) / 2) / 2048;
        temp(i+2) = ((A(4) + A(2)) / 2) / 1536;
        temp(i+3) = (A(3) - A(1))/2048;
        temp(i+4) = (A(4) - A(2))/1536;
    end
    
    path = 'C:\Users\hansd0118\Desktop\filetest\';
    writefile_name = strcat(path,file_name);
    writefile = fopen(writefile_name,'w');
    formatSpec1 = '%d %f %f %f %f\n';
    fprintf(writefile,formatSpec1,temp);
    fclose(writefile);
    fclose(readfile);
end
%%
clear all; close all; clc;
for k = 17:928
    file_name = sprintf('fc2_save_2017-01-10-144215-%04d.jpg',k);
    path = 'C:\Users\hansd0118\Desktop\';
    writefile_name = strcat(path,'train.txt');
    writefile = fopen(writefile_name,'a');
    formatSpec = '%s\n';
    file_path = strcat('data/img/',file_name);
    fprintf(writefile,formatSpec,file_path);
    fclose(writefile);
end