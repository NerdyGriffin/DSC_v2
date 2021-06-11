function receiveSerialData(app)
%RECEIVESERIALDATA Summary of this function goes here
%   Detailed explanation goes here
startDateTime = datetime;
mkdir('autosave');
matfileName = ['autosave/autoSaveData-',datestr(startDateTime, 'yyyy-mm-dd-HHMM'),'.mat'];

elapsedTime = zeros(1,app.PlotRefreshDelay);
targetTemp = zeros(1,app.PlotRefreshDelay);
refTemp = zeros(1,app.PlotRefreshDelay);
sampTemp = zeros(1,app.PlotRefreshDelay);
refCurrent = zeros(1,app.PlotRefreshDelay);
sampCurrent = zeros(1,app.PlotRefreshDelay);
refHeatFlow = zeros(1,app.PlotRefreshDelay);
sampHeatFlow = zeros(1,app.PlotRefreshDelay);
refDutyCycle = zeros(1,app.PlotRefreshDelay);
sampDutyCycle = zeros(1,app.PlotRefreshDelay);

dataLength = 0;

experimentIsRunning = true;
while experimentIsRunning
    serialData = read(app.Arduino, 1, 'char');
    
    switch serialData
        case 'd'
            readline(app.Arduino);
            dataLength = dataLength + 1;
            elapsedTime(dataLength) = double(readline(app.Arduino));
            targetTemp(dataLength) = double(readline(app.Arduino));
            refTemp(dataLength) = double(readline(app.Arduino));
            sampTemp(dataLength) = double(readline(app.Arduino));
            refCurrent(dataLength) = double(readline(app.Arduino));
            sampCurrent(dataLength) = double(readline(app.Arduino));
            refHeatFlow(dataLength) = double(readline(app.Arduino));
            sampHeatFlow(dataLength) = double(readline(app.Arduino));
            refDutyCycle(dataLength) = double(readline(app.Arduino));
            sampDutyCycle(dataLength) = double(readline(app.Arduino));
            
            if ~mod(dataLength, app.DataRefreshDelay)
                updateLiveData(app, ...
                    elapsedTime(dataLength), ...
                    targetTemp(dataLength), ...
                    refTemp(dataLength), ...
                    sampTemp(dataLength), ...
                    refCurrent(dataLength), ...
                    sampCurrent(dataLength), ...
                    refDutyCycle(dataLength), ...
                    sampDutyCycle(dataLength));
            end
            
            if ~mod(dataLength, app.PlotRefreshDelay)
                refreshLivePlot(app, elapsedTime,...
                    targetTemp, refTemp, sampTemp);
            end
        case 'x'
            experimentIsRunning = false;
            disp('Received end signal')
        otherwise
            disp('Unrecognized data flag:')
            disp(serialData)
            disp(readline(app.Arduino))
    end
end

save(matfileName, 'startDateTime', 'elapsedTime', 'targetTemp', ...
    'refTemp', 'sampTemp', ...
    'refCurrent', 'sampCurrent', ...
    'refHeatFlow', 'sampHeatFlow', ...
    'refDutyCycle', 'sampDutyCycle', 'dataLength')

updateLiveData(app, ...
    elapsedTime(dataLength), ...
    targetTemp(dataLength), ...
    refTemp(dataLength), ...
    sampTemp(dataLength), ...
    refCurrent(dataLength), ...
    sampCurrent(dataLength), ...
    refDutyCycle(dataLength), ...
    sampDutyCycle(dataLength));

refreshLivePlot(app, elapsedTime, targetTemp, refTemp, sampTemp);

setIdleUI(app);
end