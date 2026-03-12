clear; clc; close all;

%% 1. Parametry Kinematyczne (DH) i budowa robota
DH_params = [
    0    , pi/2, 0.165, 0;  % 1. Baza
    0.153, 0   , 0.05 , 0;  % 2. Bark
    0.175, 0   , 0.013, 0;  % 3. Łokieć
    0    , 0   , 0    , 0   % 4. Punkt końcowy (brak chwytaka)
];

Robot3R = rigidBodyTree("DataFormat","column");

% Baza (Serwo 1)
bark = rigidBody('bark');
staw1 = rigidBodyJoint('staw1', 'revolute');
setFixedTransform(staw1, DH_params(1,:), 'dh');
staw1.PositionLimits = [-pi/2, pi/2]; % Ograniczenie serwa do +/- 90 stopni
bark.Joint = staw1;
addBody(Robot3R, bark, 'base');

% Bark (Serwo 2)
lokiec = rigidBody('lokiec');
staw2 = rigidBodyJoint('staw2', 'revolute');
setFixedTransform(staw2, DH_params(2,:), 'dh');
staw2.PositionLimits = [-pi/2, pi/2]; % Ograniczenie serwa do +/- 90 stopni
lokiec.Joint = staw2;
addBody(Robot3R, lokiec, 'bark');

% Łokieć (Serwo 3)
nadgarstek = rigidBody('nadgarstek');
staw3 = rigidBodyJoint('staw3', 'revolute');
setFixedTransform(staw3, DH_params(3,:), 'dh');
staw3.PositionLimits = [-pi/2, pi/2]; % Ograniczenie serwa do +/- 90 stopni
nadgarstek.Joint = staw3;
addBody(Robot3R, nadgarstek, 'lokiec');

% Punkt końcowy
koniec = rigidBody('koniec');
staw4 = rigidBodyJoint('staw4', 'fixed');
setFixedTransform(staw4, DH_params(4,:), 'dh');
koniec.Joint = staw4;
addBody(Robot3R, koniec, 'nadgarstek');

%% 2. Obliczenia (Kinematyka Odwrotna)
% Cel ustawiony na idealną pozycję startową dla kątów (0,0,0)
CEL = [2, 21, 15]; 
Tmat = trvec2tform(CEL);

ik = inverseKinematics('RigidBodyTree', Robot3R);
weights = [0 0 0 1 1 1]; % Ignorujemy obrót, zależy nam tylko na trafieniu w XYZ
initialGuess = [0; pi/4; -pi/4];

% Uruchomienie solvera szukającego kątów
[configSol, solInfo] = ik('koniec', Tmat, weights, initialGuess);
anglesDeg = rad2deg(configSol);

%% 3. Wypisanie wyników
fprintf('--- WYNIKI KINEMATYKI ODWROTNEJ ---\n');
fprintf('Zadany cel (X, Y, Z): [%.3f, %.3f, %.3f]\n', CEL);
fprintf('Obliczone kąty serw (Baza, Bark, Łokieć): %.2f°, %.2f°, %.2f°\n', anglesDeg);
fprintf('Status IK: %s\n', solInfo.Status);

%% 4. Rysowanie szkieletu
figure('Name', 'Kinematyka Odwrotna - Ramię 3R', 'Color', 'w');
hold on; grid on; axis equal;

% Obliczanie aktualnej pozycji każdego przegubu na podstawie wyliczonych kątów
trans_base = trvec2tform([0 0 0]); 
trans_bark = getTransform(Robot3R, configSol, 'bark');
trans_lokiec = getTransform(Robot3R, configSol, 'lokiec');
trans_nadgarstek = getTransform(Robot3R, configSol, 'nadgarstek');
trans_koniec = getTransform(Robot3R, configSol, 'koniec');

Punkty = [trans_base(1:3,4), trans_bark(1:3,4), trans_lokiec(1:3,4), ...
          trans_nadgarstek(1:3,4), trans_koniec(1:3,4)];

X = Punkty(1, :); Y = Punkty(2, :); Z = Punkty(3, :);

% Rysowanie linii robota
plot3(X, Y, Z, '-o', 'LineWidth', 4, 'MarkerSize', 8, 'MarkerFaceColor', 'b', 'Color', [0 0.4470 0.7410]); 
% Rysowanie punktu docelowego (czerwona kropka)
plot3(CEL(1), CEL(2), CEL(3), 'r.', 'MarkerSize', 30);

title('Kinematyka Odwrotna z limitami serw (\pm 90^{\circ})');
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
axis([-0.4 0.4 -0.4 0.4 0 0.6]); 
view(135, 30);
patch([-0.4 0.4 0.4 -0.4], [-0.4 -0.4 0.4 0.4], [0 0 0 0], 'k', 'FaceAlpha', 0.05, 'EdgeColor', 'none');
