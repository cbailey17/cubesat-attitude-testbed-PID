% SDSU Space Lab - MATLAB script for collecting IMU values and calculating
% motor control inputs using a PD controller and sending them to an arduino
clear;clc
%% serial port used to send data to arduino
s = serial('COM2','BaudRate',115200)
fopen(s)

%% serial port initialization of the VecNav IMU
NET.addAssembly('C:\Users\acame\Documents\SDSU\SPACE_LAB\scitech\scitech_code\vnproglib-1-1-5\vnproglib-1.1.5.0\net\bin\win64\VectorNav.dll')
import VectorNav.Sensor.*
import VectorNav.Protocol.Uart.*
z = EzAsyncData.Connect('COM1', 115200)

%% VecNav settings
br = z.Sensor.ReadBinaryOutput1()
br.AsyncMode = AsyncMode.Port1
br.RateDivisor = 100
br.CommonField = bitor(bitor(CommonGroup.TimeStartup, CommonGroup.YawPitchRoll), CommonGroup.Imu)
z.Sensor.WriteBinaryOutput1(br)

%% Calculating the control inputs required for the reaction wheels
flushinput(s)
% Projections=[-0.5 -0.5 0.5 0.5;
%              0.5 -0.5 -0.5 0.5;
%              -1/sqrt(2)*ones(1,4)];
Projections=[-0.5 0.5 0.5 -0.5;
             -0.5 -0.5 0.5 0.5;
             -1/sqrt(2)*ones(1,4)];
G=1*[10 10 10 0 0 0]';
%set_points = [0 10 10];
i=0; tic;
DATA=zeros(31,22);
roll0=z.CurrentData.YawPitchRoll.Z;
pitch0=z.CurrentData.YawPitchRoll.Y;
yaw0=z.CurrentData.YawPitchRoll.X;
accX0 = z.CurrentData.AccelerationUncompensated.X;
accY0 = z.CurrentData.AccelerationUncompensated.Y;
accZ0 = z.CurrentData.AccelerationUncompensated.Z;
set_points = [roll0 pitch0 140];

while(1)
    pause(0.2);
    i=i+1;
    Rroll=z.CurrentData.YawPitchRoll.Z ; %- roll0;
    Ppitch=z.CurrentData.YawPitchRoll.Y; % - pitch0;
    Yyaw=z.CurrentData.YawPitchRoll.X ;%- yaw0;
    [Rroll,Ppitch,Yyaw]
    accX = z.CurrentData.AccelerationUncompensated.X; %-accX0;
    accY = z.CurrentData.AccelerationUncompensated.Y; %-accY0;
    accZ = z.CurrentData.AccelerationUncompensated.Z; %-accZ0;
    
    roll=-G(1)*double(Rroll-set_points(1))-G(4)*double((0--z.CurrentData.AngularRateUncompensated.X));
    pitch=-G(2)*double(Ppitch-set_points(2))-G(5)*double((0--z.CurrentData.AngularRateUncompensated.Y));
    yaw=-G(3)*(double(Yyaw-set_points(3)))-G(6)*double((0--z.CurrentData.AngularRateUncompensated.Z));

    % roll=-G(1)*double(-Rroll-set_points(1))-G(4)*double((0--z.CurrentData.AngularRateUncompensated.X));
    % pitch=-G(2)*double(Ppitch-set_points(2))-G(5)*double((0-z.CurrentData.AngularRateUncompensated.Y));
    % yaw=-G(3)*(double(Yyaw)+13-set_points(3))-G(6)*double((0--z.CurrentData.AngularRateUncompensated.Z));

T=Projections'*([roll;pitch;yaw]);
T1= min(250, max(-250, T(1)))+250;
T2= min(250, max(-250, T(2)))+250;
T3= min(250, max(-250, T(3)))+250;
T4= min(250, max(-250, T(4)))+250;
U=Projections*[T1;T2;T3;T4];
DATA(i,:)=[i,toc,z.CurrentData.YawPitchRoll.Z,z.CurrentData.YawPitchRoll.Y,z.CurrentData.YawPitchRoll.X,...
    z.CurrentData.AngularRateUncompensated.Z,z.CurrentData.AngularRateUncompensated.Y,z.CurrentData.AngularRateUncompensated.X,U',accX,accY,accZ,T1,T2,T3,T4,T(1),T(2),T(3), T(4)];
T1s=int2three(T1);
T2s=int2three(T2);
T3s=int2three(T3);
T4s=int2three(T4);

A=['#',T1s,',',T2s,',',T3s,',',T4s];
disp(A)
fprintf(s,A);

% Plot the yaw pitch and roll data
% DATA1=cumsum(DATA(:,2));
%  AA(1)=plot((DATA1(end-30:end)),DATA(end-30:end,5)+13,'r','LineWidth',2);
% hold all
% AA(2)=plot((DATA1(end-30:end)),DATA(end-30:end,4),'g','LineWidth',2);
% AA(3)=plot((DATA1(end-30:end)),DATA(end-30:end,3),'b','LineWidth',2);
% legend('Yaw','Pitch','Roll')
% ylim([-60 60])
%        pause(0.01)
%        delete(AA)
end
%% Plot the data 
DATA = double(DATA);
figure
plot(DATA(:,1),DATA(:,3) ,'-*r', DATA(:,1),DATA(:,4),'.-b', DATA(:,1),DATA(:,5),':k','linewidth',3); %,'r',DATA(:,2),DATA(:,4),'--b',DATA(:,2),DATA(:,5),'.k')
grid on; 
set(gca,'FontSize', 14)
%title("Euler's Angles");
xlabel('Time [s]')
ylabel('Euler Angles [deg.]');
legend('roll','pitch', 'yaw');

figure
plot(DATA(:,1),DATA(:,6) ,'-*r', DATA(:,1),DATA(:,7),'.-b', DATA(:,1),DATA(:,8),':k','linewidth',3);
legend('roll rate','pitch rate', 'yaw rate');
set(gca,'FontSize', 14)
xlabel('Time [s]')
ylabel('Angular Rates [deg.]/s');
grid on;

figure
plot(DATA(:,1),DATA(:,9) ,'-*r', DATA(:,1),DATA(:,10),'.-b', DATA(:,1),DATA(:,11),':k','linewidth',3);
legend('roll','pitch', 'yaw');
set(gca,'FontSize', 14)
xlabel('Time [s]')
ylabel('Control Torques');
grid on;
