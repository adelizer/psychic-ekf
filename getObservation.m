function [z,iFeature] = getObservation(xTrue, observationNoiseFactor)
%%
global landFeatures;
global laserSensorSettings

z =[]; iFeature = []; % return negative, no detection at all by default

l = size(landFeatures,3); % get amount of landmarks

ObsNoise = observationNoiseFactor.*randn(1,2);

for i=1 :l
     
    % Lets measure the LandFeatures and add some aditive noise
    %zTemp = DoObservationModel(xVehicle,LandFeatures(:,i))+sqrt(RTrue)*ObsNoise;
    
    zTemp(1) = sqrt((landFeatures(1,i)-xTrue(1))^2+(landFeatures(2,i)-xTrue(2))^2);
    zTemp(2) = atan2(landFeatures(2,i)-xTrue(2), landFeatures(1,i)-xTrue(1)) - xTrue(3);
    zTemp = zTemp + ObsNoise;
    % Need to see witch quadrant angle (passing angle
    zTemp(2) = angleWrap(zTemp(2));
    
    %Observation model of the sensor ( Bearing and range
    if(abs(zTemp(2))<laserSensorSettings.Bearing*pi/180 && zTemp(1) < laserSensorSettings.Range)
        z=[z;zTemp]; % copy calculated observation
        iFeature =[iFeature,i]; % Return lable of the feature
        fprintf(sprintf('=======================================================================   Range is: %d, LandMark Found !! ID:%d \n',z(1),i));
    end
end;