function robotCh = connect2Robots(robot, robotCh)
%%
rootdir = cd;

cd('C:\Users\LabUser\odrive\Amazon Cloud Drive\Documents\matlab_code\file_exchange_code\sshfrommatlab_13b')
javaaddpath('ganymed-ssh2-build250.jar')

for r = 1:length(robot)    
    
    if regexp(char(robotCh{r}), 'ch.ethz\w*') == 1
        continue
    end
       
    ipAddr = [num2str(robot(r).ip(1)) '.' num2str(robot(r).ip(2)) '.' ...
        num2str(robot(r).ip(3)) '.' num2str(robot(r).ip(4))];
    robotCh{r}  =  sshfrommatlab('root', ipAddr,'');
end
cd(rootdir)
