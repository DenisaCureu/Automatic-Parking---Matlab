%% Simulator de parcare automata
% Încărcam și afișam cele trei straturi ale hărții. În fiecare strat, celulele întunecate
% reprezintă celulele ocupate, iar celulele albe reprezintă celule libere.
mapLayers = loadParkingLotMapLayers;
plotMapLayers(mapLayers)

%%
% Pentru simplitate, combinam cele trei straturi într-o singură hartă.
costmap = combineMapLayers(mapLayers);

figure
plot(costmap, 'Inflation', 'off')
legend off

%%
% Harta acoperă întreaga zonă de parcare de 75 m pe 50 m, împărțită în
% celule pătrate de 0,5 m pe 0,5 m.

costmap.MapExtent % [x, width, y, height] in meters

costmap.CellSize  % cell size in meters

%%
% Creare obiect pentru stocarea dimensiunilor vehiculului. De asemenea, definirea unghiului maxim de virare al vehiculului. Această valoare determină
% limitele razei de viraj în timpul planificării și controlului mișcării.
vehicleDims      = vehicleDimensions;
maxSteeringAngle = 35; % in degrees

%%
% Actualizarea dimensiunilor vehiculului. Adica, dimensiunea locului de
% parcare sa fie in concordanta cu dimensiunea vehiculului nostru.
costmap.CollisionChecker.VehicleDimensions = vehicleDims;

%%
% Definirea poziției de pornire a vehiculului. 
% Pozitia vehiculului este specificată ca $[x,y,\theta]$.
% $(x,y)$ reprezintă poziția centrului axei spate a vehiculului
% $\theta$ reprezintă orientarea vehiculului față de axa X.

currentPose = [4 12 0]; % [x, y, theta]

%% Partea comportamentala
% Stratul comportamental adună informații din toate părțile relevante ale sistemului, inclusiv:
%
% * Localizare: stratul comportamental inspectează modulul de localizare pentru o estimare a locației actuale a vehiculului.
% * Modelul de mediu: Sistemele de percepție și fuziune a senzorilor raportează o hartă a
% mediului din jurul vehiculului.

%%
% Încărcam fișierul routePlan.mat care conține un plan de rută care este stocat într-un tabel
% Tabelul  are trei variabile: |StartPose|, |EndPose| și |Atribute|.
% |StartPose| și |EndPose| specifica pozițiile de început și de sfârșit ale segmentului, exprimat ca $[x,y,\theta]$. 
% |Atribute| specifică proprietățile segmentului, cum ar fi limita de viteză.
data = load('routePlan.mat');
routePlan = data.routePlan %#ok<NOPTS>

% Trasează un vehicul în poziția curentă și de-a lungul fiecărui obiectiv
% din traseu.

% Plot vehicle at current pose
hold on
helperPlotVehicle(currentPose, vehicleDims, 'DisplayName', 'Current Pose')
legend

for n = 1 : height(routePlan)
    % Extract the goal waypoint
    vehiclePose = routePlan{n, 'EndPose'};
    
    % Plot the pose
    legendEntry = sprintf('Goal %i', n);
    helperPlotVehicle(vehiclePose, vehicleDims, 'DisplayName', legendEntry);
end
hold off

% Metoda requestManeuver solicită un flux de sarcini de navigare de la planificatorul comportamental până la
% destinație.
behavioralPlanner = HelperBehavioralPlanner(routePlan, maxSteeringAngle);

%% Planificarea mișcării
% Crearea unui obiect pentru a configura un traseu optim care explorează rapid abordarea arborelui aleatoriu (RRT*). 
% RRT: algoritmi care găsesc o cale prin construirea unui arbore de conexiuni. 

motionPlanner = pathPlannerRRT(costmap, 'MinIterations', 1000, ...
    'ConnectionDistance', 10, 'MinTurningRadius', 20);

% Planificarea caii de la pozitia curentă la primul obiectiv folosind functia |plan|. Calea de referinta returnată este fezabilă și fără coliziuni
goalPose = routePlan{1, 'EndPose'};
refPath = plan(motionPlanner, currentPose, goalPose);

% Calea de referință constă dintr-o secvență de segmente. Fiecare segment
% descrie setul de manevre Dubins sau Reeds-Shepp.
refPath.PathSegments

% Preluam pozițiile și direcțiile de tranziție din calea planificată.
[transitionPoses, directions] = interpolate(refPath);

% Vizualizarea caii planificată.
plot(motionPlanner)

%% Aranjarea traseului și generarea traiectoriei
% Calea de referință generată de planificatorul de trasee este compusă din:
% Dubins sau Reeds-Shepp. Pentru a evita miscarile bruste și pentru a asigura confortul pasagerului, 
% calea trebuie să fie libera. 

% Specificarea numărului de pozitie returnat folosind o separare de aproximativ 0,1 m.
approxSeparation = 0.1; % meters
numSmoothPoses   = round(refPath.Length / approxSeparation);

% Returnează pozițiile discretizate de-a lungul căii netede.
[refPoses, directions, cumLengths, curvatures] = smoothPathSpline(transitionPoses, directions, numSmoothPoses);

% Trasarea caii netezite.
hold on
hSmoothPath = plot(refPoses(:, 1), refPoses(:, 2), 'r', 'LineWidth', 2, ...
    'DisplayName', 'Smoothed Path');
hold off

p.
maxSpeed   = 5; % in meters/second
startSpeed = 0; % in meters/second
endSpeed   = 0; % in meters/second

%%
% Generam un profil de viteză.
refVelocities = helperGenerateVelocityProfile(directions, cumLengths, curvatures, startSpeed, endSpeed, maxSpeed);

%%
% |refVelocities| conține viteze de referință pentru fiecare punct de-a lungul caii netezite.
plotVelocityProfile(cumLengths, refVelocities, maxSpeed)


% * *Control lateral*: Reglarea unghiului de virare astfel încât vehiculul
% sa urmeze calea de referință.
% * *Control longitudinal*: În timp ce urmeaza calea de referință, menține
% viteza dorită prin controlul clapetei de accelerație și a frânei.
% Acest scenariu implică viteze mici.
% $$\dot x_r = v_r*\cos(\theta) $$
%
% $$\dot y_r = v_r*\sin(\theta) $$
%
% $$\dot \theta = \frac{v_r}{l}*\tan(\delta) $$
%
% $$\dot v_r = a_r $$
%
% În ecuațiile de mai sus, $(x_r,y_r,\theta)$ reprezintă poziția vehiculului în coordonate mondiale. 
% $v_r$, $a_r$, $l$ și $\delta$ reprezintă viteza roții din spate,
% accelerația roților din spate, ampatamentul și unghiul de virare.
% Poziția și viteza roții din față pot fi obținute:
%
% $$ x_f = x_r + l cos(\theta)$$
%
% $$ y_f = y_r + l sin(\theta)$$
%
% $$ v_f = \frac{v_r} {cos(\delta)}$$
%
% Inchidem toate figurile
closeFigures;

% Crearea simulatorului.
vehicleSim = HelperVehicleSimulator(costmap, vehicleDims);

% Setarea pozitiei si vitezei vechiculului.
vehicleSim.setVehiclePose(currentPose);
currentVel = 0;
vehicleSim.setVehicleVelocity(currentVel);

% Configuram simulatorul pentru a ne arata traiectoria.
vehicleSim.showTrajectory(true);

% Ascunde figura simulatorului vehiculului.
hideFigure(vehicleSim);

%%
% Crearea unui obiect pentru a calcula poziția de referință, viteza de referință și direcția de mers pentru controler.
pathAnalyzer = HelperPathAnalyzer(refPoses, refVelocities, directions, ...
    'Wheelbase', vehicleDims.Wheelbase);

%% 
% Crearea unui obiect pentru a controla viteza și timpul de probă.
sampleTime = 0.05;
lonController = HelperLongitudinalController('SampleTime', sampleTime);

%%
% Utilizam obiectul pentru a asigura execuția cu rată fixă ​​a controler'ului. 
% Utilizam o rată de control pentru a fi în concordanță cu cea longitudinală
controlRate = HelperFixedRate(1/sampleTime); % in Hertz

% * Calculam comenzile de direcție și accelerare/decelerare necesare pentru traiectoria planificată.
% * Introducem comenzile de control la simulator.
% * Înregistram poziția și viteza vehiculului.
reachGoal = false;

while ~reachGoal    
    % Gasim pozitia de referinta a caii si viteza corespunzatoare.
    [refPose, refVel, direction] = pathAnalyzer(currentPose, currentVel);
    
    % Actualizam directia vehiculului pentru simulator.
    updateDrivingDirection(vehicleSim, direction);
    
    % Calculam comanda de direcție.
    steeringAngle = lateralControllerStanley(refPose, currentPose, currentVel, ...
        'Direction', direction, 'Wheelbase', vehicleDims.Wheelbase);
    
    % Calculam comanda de acceleratia si deceleratia.
    lonController.Direction = direction;
    [accelCmd, decelCmd] = lonController(refVel, currentVel);
    
    % Simularea vehiculului utilizând ieșirile controlerului.
    drive(vehicleSim, accelCmd, decelCmd, steeringAngle);
    
    % Verificam daca vehiculul ajunge la destinatie.
    reachGoal = helperGoalChecker(goalPose, currentPose, currentVel, endSpeed, direction);
    
    % Asteptam pentru executia ratei fixe.
    waitfor(controlRate);
    
    % Luam pozitia si viteza vehiculului.
    currentPose  = getVehiclePose(vehicleSim);
    currentVel   = getVehicleVelocity(vehicleSim);
end

% Figura simulatorului.
showFigure(vehicleSim);

%% Execute a Complete Plan
% Acum combinam toti pasii de mai sus.
% Setarea pozitiei vehiculului la punctul de start.
currentPose = [4 12 0]; % [x, y, theta]
vehicleSim.setVehiclePose(currentPose);

% Resetare viteza.
currentVel  = 0; % meters/second
vehicleSim.setVehicleVelocity(currentVel);

while ~reachedDestination(behavioralPlanner)
    
    % Solicitam următoarea manevră de la stratul comportamental.
    [nextGoal, plannerConfig, speedConfig] = requestManeuver(behavioralPlanner, ...
        currentPose, currentVel);
    
    % Configuram planificatorul de mișcare.
    configurePlanner(motionPlanner, plannerConfig);
    
    % Planificam o cale de referință folosind planificatorul RRT* la următoarea poziție a obiectivului.
    refPath = plan(motionPlanner, currentPose, nextGoal);
    
    % Verificam dacă calea este validă. Dacă planificatorul nu reușește să calculeze o cale,
    % sau calea nu este fără coliziuni din cauza actualizărilor hărții,
    % sistemul trebuie replanificat. Acest scenariu folosește o hartă statică, deci calea
    % va fi întotdeauna fără coliziuni.
    isReplanNeeded = ~checkPathValidity(refPath, costmap);
    if isReplanNeeded
        warning('Unable to find a valid path. Attempting to re-plan.')
        
        % Solicitam planificatorului comportamental să replanifice
        replanNeeded(behavioralPlanner);
        continue;
    end
    
    % Preluam pozițiile și direcțiile de tranziție din calea planificată.
    [transitionPoses, directions] = interpolate(refPath);
     
    % Netezim calea.
    numSmoothPoses   = round(refPath.Length / approxSeparation);
    [refPoses, directions, cumLengths, curvatures] = smoothPathSpline(transitionPoses, directions, numSmoothPoses);
    
    % Generam un profil de viteză.
    refVelocities = helperGenerateVelocityProfile(directions, cumLengths, curvatures, startSpeed, endSpeed, maxSpeed);
    
    % Configuram analizorul de cale.
    pathAnalyzer.RefPoses     = refPoses;
    pathAnalyzer.Directions   = directions;
    pathAnalyzer.VelocityProfile = refVelocities;
    
    % Resetam controlerul longitudinal.
    reset(lonController);
    
    reachGoal = false;

    while ~reachGoal  
       
        [refPose, refVel, direction] = pathAnalyzer(currentPose, currentVel);
        

        updateDrivingDirection(vehicleSim, direction);
       
        steeringAngle = lateralControllerStanley(refPose, currentPose, currentVel, ...
            'Direction', direction, 'Wheelbase', vehicleDims.Wheelbase);
        
        
        lonController.Direction = direction;
        [accelCmd, decelCmd] = lonController(refVel, currentVel);
        
        drive(vehicleSim, accelCmd, decelCmd, steeringAngle);
        

        reachGoal = helperGoalChecker(nextGoal, currentPose, currentVel, speedConfig.EndSpeed, direction);
      
        waitfor(controlRate);
        
        currentPose  = getVehiclePose(vehicleSim);
        currentVel   = getVehicleVelocity(vehicleSim);
    end
end

showFigure(vehicleSim);


hideFigure(vehicleSim);



figure
plot(ccConfig)
title('Current Collision Checker')




ccConfig.NumCircles = 4;

figure
plot(ccConfig)
title('New Collision Checker')

 
costmap.CollisionChecker = ccConfig;


figure
plot(costmap)
title('Costmap with updated collision checker')

parkMotionPlanner = pathPlannerRRT(costmap, 'MinIterations', 1000);


parkPose = [36 44 90];
preParkPose = currentPose;


refPath = plan(parkMotionPlanner, preParkPose, parkPose);


figure
plotParkingManeuver(costmap, refPath, preParkPose, parkPose)



[transitionPoses, directions] = interpolate(refPath);

h.
numSmoothPoses   = round(refPath.Length / approxSeparation);
[refPoses, directions, cumLengths, curvatures] = smoothPathSpline(transitionPoses, directions, numSmoothPoses);


refVelocities = helperGenerateVelocityProfile(directions, cumLengths, curvatures, currentVel, 0, 2.2352);

pathAnalyzer.RefPoses     = refPoses;
pathAnalyzer.Directions   = directions;
pathAnalyzer.VelocityProfile = refVelocities;


reset(lonController);

reachGoal = false;

while ~reachGoal 
   
    [refPose, refVel, direction] = pathAnalyzer(currentPose, currentVel);
    

    updateDrivingDirection(vehicleSim, direction);
    

    steeringAngle = lateralControllerStanley(refPose, currentPose, currentVel, ...
        'Direction', direction, 'Wheelbase', vehicleDims.Wheelbase);

    lonController.Direction = direction;
    [accelCmd, decelCmd] = lonController(refVel, currentVel);

    drive(vehicleSim, accelCmd, decelCmd, steeringAngle);
    
    
    reachGoal = helperGoalChecker(parkPose, currentPose, currentVel, 0, direction);
    
    waitfor(controlRate);
    

    currentPose  = getVehiclePose(vehicleSim);
    currentVel   = getVehicleVelocity(vehicleSim);
end


closeFigures;
showFigure(vehicleSim);



parkPose = [49 47.2 -90];


parkMotionPlanner.ConnectionMethod = 'Reeds-Shepp';


parkMotionPlanner.MinTurningRadius   = 10; % meters
parkMotionPlanner.ConnectionDistance = 15;


currentVel = 0;
vehicleSim.setVehiclePose(preParkPose);
vehicleSim.setVehicleVelocity(currentVel);


replan = true;
while replan
    refPath = plan(parkMotionPlanner, preParkPose, parkPose);
    
   
    
    numSamples = 10;
    stepSize   = refPath.Length / numSamples;
    lengths    = 0 : stepSize : refPath.Length;
    
    [transitionPoses, directions] = interpolate(refPath, lengths);


    replan = sum(abs(diff(directions)))~=2 || refPath.Length > 20;
end


figure
plotParkingManeuver(costmap, refPath, preParkPose, parkPose)

numSmoothPoses   = round(refPath.Length / approxSeparation);
[refPoses, directions, cumLengths, curvatures] = smoothPathSpline(transitionPoses, directions, numSmoothPoses, 0.5);
    

refVelocities = helperGenerateVelocityProfile(directions, cumLengths, curvatures, currentVel, 0, 1);

pathAnalyzer.RefPoses     = refPoses;
pathAnalyzer.Directions   = directions;
pathAnalyzer.VelocityProfile = refVelocities;

reset(lonController);

reachGoal = false;

while ~reachGoal    
  
    currentDir = getDrivingDirection(vehicleSim);
   
    [refPose, refVel, direction] = pathAnalyzer(currentPose, currentVel);
    

    if currentDir ~= direction
        currentVel = 0;
        setVehicleVelocity(vehicleSim, currentVel);
        reset(lonController);
    end
   
    currentVel = updateDrivingDirection(vehicleSim, direction, currentDir);
 
    steeringAngle = lateralControllerStanley(refPose, currentPose, currentVel, ...
        'Direction', direction, 'Wheelbase', vehicleDims.Wheelbase);
    

    lonController.Direction = direction;
    [accelCmd, decelCmd] = lonController(refVel, currentVel);
    
 
    drive(vehicleSim, accelCmd, decelCmd, steeringAngle);
    
 
    reachGoal = helperGoalChecker(parkPose, currentPose, currentVel, 0, direction);

    waitfor(controlRate);
    

    currentPose  = getVehiclePose(vehicleSim);
    currentVel   = getVehicleVelocity(vehicleSim);
end


closeFigures;
snapnow;

% Delete the simulator.
delete(vehicleSim);

function mapLayers = loadParkingLotMapLayers()


mapLayers.StationaryObstacles = imread('stationary.bmp');
mapLayers.RoadMarkings        = imread('road_markings.bmp');
mapLayers.ParkedCars          = imread('parked_cars.bmp');
end





figure
cellOfMaps = cellfun(@imcomplement, struct2cell(mapLayers), 'UniformOutput', false);
montage( cellOfMaps, 'Size', [1 numel(cellOfMaps)], 'Border', [5 5], 'ThumbnailSize', [300 NaN] )
title('Map Layers - Stationary Obstacles, Road markings, and Parked Cars')
end


function costmap = combineMapLayers(mapLayers)


combinedMap = mapLayers.StationaryObstacles + mapLayers.RoadMarkings + ...
    mapLayers.ParkedCars;
combinedMap = im2single(combinedMap);

res = 0.5; % meters
costmap = vehicleCostmap(combinedMap, 'CellSize', res);
end


function configurePlanner(pathPlanner, config)


fieldNames = fields(config);
for n = 1 : numel(fieldNames)
    if ~strcmpi(fieldNames{n}, 'IsParkManeuver')
        pathPlanner.(fieldNames{n}) = config.(fieldNames{n});
    end
end
end

%%%
% *plotVelocityProfile*
% Plot speed profile
function plotVelocityProfile(cumPathLength, refVelocities, maxSpeed)



plot(cumPathLength, refVelocities, 'LineWidth', 2);

hold on
line([0;cumPathLength(end)], [maxSpeed;maxSpeed], 'Color', 'r')
hold off

buffer = 2;
xlim([0 cumPathLength(end)]);
ylim([0 maxSpeed + buffer])


xlabel('Cumulative Path Length (m)');
ylabel('Velocity (m/s)');

legend('Velocity Profile', 'Max Speed')
title('Generated velocity profile')
end


function closeFigures()


figHandles = findobj('Type', 'figure');
for i = 1: length(figHandles)
    if ~strcmp(figHandles(i).Name, 'Automated Valet Parking')
        close(figHandles(i));
    end
end
end

%%%
% *plotParkingManeuver*
% Display the generated parking maneuver on a costmap.
function plotParkingManeuver(costmap, refPath, currentPose, parkPose)
%plotParkingManeuver
% Plot the generated parking maneuver on a costmap.

% Plot the costmap, without inflated areas.
plot(costmap, 'Inflation', 'off')


hold on
plot(refPath, 'DisplayName', 'Parking Maneuver')

title('Parking Maneuver')


lo = min([currentPose(1:2); parkPose(1:2)]);
hi = max([currentPose(1:2); parkPose(1:2)]);

buffer = 6; % meters

xlim([lo(1)-buffer hi(1)+buffer])
ylim([lo(2)-buffer hi(2)+buffer])
end