clear all;
close all;
clc;
vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

JOINT_ID = [0,0,0,0,0,0];
Cam_ID = 0;

fig1 = figure('Position', [100, 200, 640, 480]);

FrameRate = 0;
elapsedTime = 0;
t = 0;

if (clientID>-1)
    disp('Connected to remote API server');
    
    %[number returnCode,number handle]=simxGetObjectHandle(number clientID,string objectName,number operationMode)
    [~,JOINT_ID(1)] = vrep.simxGetObjectHandle(clientID, 'J1_PSM1', vrep.simx_opmode_blocking);
    [~,JOINT_ID(2)] = vrep.simxGetObjectHandle(clientID, 'J2_PSM1', vrep.simx_opmode_blocking);
    [~,JOINT_ID(3)] = vrep.simxGetObjectHandle(clientID, 'J3_PSM1', vrep.simx_opmode_blocking);
    [~,JOINT_ID(4)] = vrep.simxGetObjectHandle(clientID, 'J1_TOOL1', vrep.simx_opmode_blocking);
    [~,JOINT_ID(5)] = vrep.simxGetObjectHandle(clientID, 'J2_TOOL1', vrep.simx_opmode_blocking);
    [~,JOINT_ID(6)] = vrep.simxGetObjectHandle(clientID, 'J3_dx_TOOL1', vrep.simx_opmode_blocking);
    [~,JOINT_ID(7)] = vrep.simxGetObjectHandle(clientID, 'J3_sx_TOOL1', vrep.simx_opmode_blocking);
    [~,Cam_ID]=vrep.simxGetObjectHandle(clientID,'BillyCam',vrep.simx_opmode_blocking);
    
    dVRK_pos_init = [0,0,0,0,0,0];
    dVRK_pos = [0,0,0,0,0,0];
    
    for j=1:2  %must do GetJointPosition twice to get valid data (bug)
        for i=1:7
            %[number returnCode,number position]=simxGetJointPosition(number clientID,number jointHandle,number operationMode)
            [~,dVRK_pos_init(i)] = vrep.simxGetJointPosition(clientID, JOINT_ID(i), vrep.simx_opmode_streaming);
            %must pause otherwise data invalid
            pause(0.01);
        end
    end
    
    while true
        
        tic
        
        %[number returnCode,array resolution,matrix image]=simxGetVisionSensorImage2(number clientID,number sensorHandle,number options,number operationMode)
        [~,img_res,img] = vrep.simxGetVisionSensorImage2(clientID, Cam_ID, 1, vrep.simx_opmode_blocking);
        img = flip(img);
        img = flip(img,2);
        
        %Get joint positions
        for i=1:7
            %[number returnCode,number position]=simxGetJointPosition(number clientID,number jointHandle,number operationMode)
            [~,dVRK_pos(i)] = vrep.simxGetJointPosition(clientID, JOINT_ID(i), vrep.simx_opmode_streaming);
        end 
        
        %Assign new joint positions
        dVRK_pos(5) = sin(0.01*pi*t);
        dVRK_pos_init
        
        for i=1:7
            %[number returnCode]=simxSetJointPosition(number clientID,number jointHandle,number position,number operationMode)
            vrep.simxSetJointPosition(clientID, JOINT_ID(i), dVRK_pos(i), vrep.simx_opmode_oneshot);
        end
        
%         [~,Ang] = vrep.simxGetObjectOrientation(clientID, JOINT_ID(7), JOINT_ID(1), vrep.simx_opmode_oneshot);
%         Ang
        set(0, 'CurrentFigure', fig1);
        cla reset;
        imshow(img);
        hold on;
        visual = sprintf('Res: %d*%d, FrameRate: %.1f fps', img_res(1), img_res(2), FrameRate);
        title(visual);
        hold off;
        
        if strcmpi(get(fig1,'CurrentKey'),'q')
            close all;
            break;
        end

        pause(0.01);
        
        t = t + 1;
        
        elapsedTime = toc;
        FrameRate = 1/elapsedTime;
    end
    
else
    disp('Failed connecting to remote API server');
end
vrep.delete(); % call the destructor!

disp('Program ended');