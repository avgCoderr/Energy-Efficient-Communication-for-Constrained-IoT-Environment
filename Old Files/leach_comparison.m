%Rishabh Barnwal%
%2020A7PS1677P%
%2023%
close all;
clear;
clc;
%%%%%%%%%%%%%%%%%%%% Network Establishment Parameters %%%%%%%%%%%%%%%%%%%%
%%% plot of Operation %%%
% Field Dimensions in meters %
% Define the grid size
x_max = 1000;
y_max = 1000;
x_min = 10;
y_min = 10;
step = 100;

% Generate the grid
x_cord = x_min:step:(x_min + (x_max - 1) * step);
y_cord = y_min:step:(y_min + (y_max - 1) * step);

x = 0; % added for better display results of the plot
y = 0; % added for better display results of the plot
% Number of Nodes in the field %
n = 100;
% Number of Dead Nodes in the beggining %
dead_nodes = 0;
% Coordinates of the Sink (location is predetermined in this simulation) %
sinkx = 990;
sinky = 990;
%%% Energy Values %%%
% Initial Energy of a Node (in Joules) %
Eo = 0.1; % units in Joules
% Energy required to run circuity (both for transmitter and receiver) %
Eelec = 50 * 10 ^ (-9); % units in Joules/bit
ETx = 50 * 10 ^ (-9); % units in Joules/bit
ERx = 50 * 10 ^ (-9); % units in Joules/bit
% Transmit Amplifier Types %
Eamp = 100 * 10 ^ (-12); % units in Joules/bit/m^2 (amount of energy spent by the amplifier to transmit the bits)
% Data Aggregation Energy %
EDA = 5 * 10 ^ (-9); % units in Joules/bit
% Size of data package %
k = 4000; % units in bits
% Suggested percentage of cluster head %
p = 0.1; % a 1 percent of the total amount of nodes used in the network is proposed to give good results
% Number of Clusters %
Ci = p * n;
% Round of Operation %
rounds = 0;
% Current Number of operating Nodes %
totalEnergy = n * Eo;
operating_nodes = n;
transmissions = 0;
temp_val = 0;
flagFirstDead = 0;
stop_flag = 0;

%%% Creation of the Wireless Sensor Network %%%
% Plotting the WSN %
i = 1;

while (i <= n)

    for j = 1:(x_max / step)

        for k = 1:(y_max / step)
            SN(i).x = x_cord(j); % X-axis coordinates of sensor node
            SN(i).y = y_cord(k); % Y-axis coordinates of sensor node
            i = i + 1;
        end

    end

end

for i = 1:n

    SN(i).id = i; % sensor's ID number
    SN(i).Energy = Eo; % nodes energy levels (initially set to be equal to "Eo"
    SN(i).role = 0; % node acts as normal if the value is '0', if elected as a cluster head it gets the value '1' (initially all nodes are normal)
    SN(i).cluster = 0; % the cluster which a node belongs to
    SN(i).condition = 1; % States the current condition of the node. when the node is operational its value is =1 and when dead =0
    SN(i).rop = 0; % number of rounds node was operational
    SN(i).rleft = 0; % rounds left for node to become available for Cluster Head election
    SN(i).dtch = 0; % nodes distance from the cluster head of the cluster in which he belongs
    SN(i).dts = sqrt((sinkx - SN(i).x) ^ 2 + (sinky - SN(i).y) ^ 2); % nodes distance from the sink
    SN(i).tel = 0; % states how many times the node was elected as a Cluster Head
    SN(i).rn = 0; % round node got elected as cluster head
    SN(i).chid = 0; % node ID of the cluster head which the "i" normal node belongs to
    SN(i).prob = p; % probability with which a node gets to be cluster head

    hold on;
    figure(1);
    plot(x, y, x_max, y_max, SN(i).x, SN(i).y, 'ob', sinkx, sinky, '*r');
    title 'Uniform Wireless Sensor Network';
    xlabel 'X-coordinate (Field size in meters)';
    ylabel 'Y-coordinate (Field size in meters)';

end

CHeads = 0;
CH_list = [];
firstRound = 1;
totalData = 0;
networkStatus = 1;
rad = 200;
dead_nodes = 0;
rounds = 0;

opNodes = [];
dNodes = [];
cNodes = [];
threshData = [];
remainingEnergy = [];
nrg = [];
flagFirstDead = 0;
networkLiveData_1 = [];
abpl_1 = [];
lsp_1 = [];

%%%%%% Set-Up Phase %%%%%%
%%%%% PROPOSED WITH DEGREE %%%%
while (operating_nodes > 0 && stop_flag == 0)

    % Displays Current Round %
    rounds = rounds + 1
    remainingEnergy(rounds) = totalEnergy;

    % Reseting Previous Amount Of Energy Consumed In the Network on the Previous Round %
    energy = 0;

    % New list of nodes is sorted on basis of energy %
    SN = sortNodes_energy(SN, n);

    % Cluster Heads Election %
    [SN, CH_list, CHeads, firstRound] = maxE_minD_selection_new(SN, n, Ci, Eo, CH_list, CHeads, firstRound, operating_nodes);

    if (CHeads ~= 0)
        % Fixing the size of "CL" array %
        CH_list = CH_list(1:CHeads);
        CH_list = arrayfun(@(x) setfield(x, 'degree', 0), CH_list);

        % Grouping the Nodes into Clusters & caclulating the distance between node and cluster head %
        for i = 1:n

            if ((SN(i).role == 0) && (SN(i).Energy > 0)) % if node is normal

                for m = 1:CHeads
                    d(m) = sqrt((CH_list(m).x - SN(i).x) ^ 2 + (CH_list(m).y - SN(i).y) ^ 2);
                    % we calculate the distance 'd' between the sensor node that is
                    % transmitting and the cluster head that is receiving with the following equation+
                    % d=sqrt((x2-x1)^2 + (y2-y1)^2) where x2 and y2 the coordinates of
                    % the cluster head and x1 and y1 the coordinates of the transmitting node
                end

                d = d(1:CHeads); % fixing the size of "d" array
                [M, I] = min(d(:)); % finds the minimum distance of node to CH
                [Row, Col] = ind2sub(size(d), I); % displays the Cluster Number in which this node belongs too
                SN(i).cluster = Col; % assigns node to the cluster
                SN(i).dtch = d(Col); % assigns the distance of node to CH
                SN(i).chid = CH_list(Col).id;
                CH_list(Col).degree = CH_list(Col).degree + 1;
            end

        end

        % printing the degree of each CH %

        % calculating HOPS and Proximity %
        [CH_list, ABPL] = CH_list_with_HOPS(CH_list, CHeads, rad, rounds);
        Network_Longest_Shortest_Path = calculating_lsp(CH_list, CHeads);

        abpl_1(rounds) = ABPL;
        lsp_1(rounds) = Network_Longest_Shortest_Path;

        %%%%%% Steady-State Phase %%%%%%
        % Energy Dissipation for normal nodes %

        for i = 1:n

            if (SN(i).condition == 1) && (SN(i).role == 0)

                if SN(i).Energy > 0
                    ETx = Eelec * k + Eamp * k * SN(i).dtch ^ 2;
                    SN(i).Energy = SN(i).Energy - ETx;
                    energy = energy + ETx;
                    totalEnergy = totalEnergy - ETx;
                    totalData = totalData + k;

                    % Dissipation for cluster head during reception
                    if SN(SN(i).chid).Energy > 0 && SN(SN(i).chid).condition == 1 && SN(SN(i).chid).role == 1
                        ERx = (Eelec + EDA) * k;
                        energy = energy + ERx;
                        totalEnergy = totalEnergy - ERx;
                        SN(SN(i).chid).Energy = SN(SN(i).chid).Energy - ERx;

                        if SN(SN(i).chid).Energy <= 0 % if cluster heads energy depletes with reception
                            SN(SN(i).chid).condition = 0;
                            SN(SN(i).chid).rop = rounds;
                            dead_nodes = dead_nodes + 1;
                            operating_nodes = operating_nodes - 1
                        end

                    end

                end

                if SN(i).Energy <= 0 % if nodes energy depletes with transmission
                    dead_nodes = dead_nodes + 1;
                    operating_nodes = operating_nodes - 1
                    SN(i).condition = 0;
                    SN(i).chid = 0;
                    SN(i).rop = rounds;
                end

            end

        end

        % Energy Dissipation for cluster head nodes %

        for i = 1:n

            if (SN(i).condition == 1) && (SN(i).role == 1)

                if SN(i).Energy > 0
                    ETx = (Eelec + EDA) * k + Eamp * k * SN(i).dts ^ 2;
                    SN(i).Energy = SN(i).Energy - ETx;
                    energy = energy + ETx;
                    totalEnergy = totalEnergy - ETx;
                end

                if SN(i).Energy <= 0 % if cluster heads energy depletes with transmission
                    dead_nodes = dead_nodes + 1;
                    operating_nodes = operating_nodes - 1
                    SN(i).condition = 0;
                    SN(i).rop = rounds;
                end

            end

        end

    end

    % Obtaining network status %
    if operating_nodes < n && temp_val == 0
        temp_val = 1;
        flagFirstDead = rounds;
        networkLiveData_1(networkStatus) = rounds;
        networkStatus = networkStatus + 1;
    end

    if operating_nodes <= n * 0.5 && temp_val == 1
        temp_val = 2;
        networkLiveData_1(networkStatus) = rounds;
        networkStatus = networkStatus + 1;
    end

    if flagFirstDead == 0
        transmissions = transmissions + 1;
        nrg(transmissions) = energy;
    end

    if CHeads == 0 || operating_nodes < 3
        transmissions = transmissions - 1;
        stop_flag = 1;
        CHeads = 0;
        networkLiveData_1(networkStatus) = rounds;
    end

    opNodes(rounds) = operating_nodes;
    dNodes(rounds) = dead_nodes;
    cNodes(rounds) = CHeads;
    threshData(rounds) = totalData;

end

% Plotting Simulation Results "Operating Nodes per Round" %
figure(2)
plot(1:length(opNodes), opNodes(1:length(opNodes)), '-r', 'Linewidth', 2);
title ({'COMPARISION'; 'Number of Operating Nodes per Round'; })
xlabel '# Rounds';
ylabel '# Operational Nodes';
hold on;

figure(3)
plot(1:length(dNodes), dNodes(1:length(dNodes)), '-r', 'Linewidth', 2);
title ({'COMPARISION'; 'Number of Dead Nodes per Round'; })
xlabel '# Rounds';
ylabel '# Dead Nodes';
hold on;

figure(4)
plot(1:length(cNodes), cNodes(1:length(cNodes)), '-r', 'Linewidth', 2);
title ({'COMPARISION'; 'Number of Clusters per Round'; })
xlabel '# Rounds';
ylabel '# Clusters';
hold on;

figure(5)
plot(1:length(threshData), threshData(1:length(threshData)), '-r', 'Linewidth', 2);
title ({'COMPARISION'; 'Data Packets Sent to the Sink Node per Round'; })
xlabel '# Rounds';
ylabel 'Data Packets Sent to the Sink Node';
hold on;

figure(6)
plot(1:length(nrg), nrg(1:length(nrg)), '-r', 'Linewidth', 2);
title ({'COMPARISION'; 'Energy Consumtion per Transmission'; })
xlabel '# Transmission';
ylabel 'Energy (Joule)';
hold on;

figure(7)
area(1:length(remainingEnergy), remainingEnergy(1:length(remainingEnergy)), 'FaceColor', 'r');
title ({'COMPARISION'; 'Remaining Energy per Round'; })
xlabel '# Rounds';
ylabel 'Remaining Energy (Joule)';
hold on;

figure(8)
plot(1:length(abpl_1), abpl_1(1:length(abpl_1)), '-r', 'Linewidth', 2);
title ({'COMPARISION'; 'Average Backbone Path Length per Round'; })
xlabel '# Rounds';
ylabel 'Average Backbone Path Length';
hold on;

figure(9)
plot(1:length(lsp_1), lsp_1(1:length(lsp_1)), '-r', 'Linewidth', 2);
title ({'COMPARISION'; 'Longest Shortest Path Length per Round'; })
xlabel '# Rounds';
ylabel 'Longest Shortest Path Length';
hold on;

%-----------Re-initializing values-----------%
dead_nodes = 0;
% Coordinates of the Sink (location is predetermined in this simulation) %
sinkx = 990;
sinky = 990;
%%% Energy Values %%%
% Initial Energy of a Node (in Joules) %
Eo = 0.1; % units in Joules
% Energy required to run circuity (both for transmitter and receiver) %
Eelec = 50 * 10 ^ (-9); % units in Joules/bit
ETx = 50 * 10 ^ (-9); % units in Joules/bit
ERx = 50 * 10 ^ (-9); % units in Joules/bit
% Transmit Amplifier Types %
Eamp = 100 * 10 ^ (-12); % units in Joules/bit/m^2 (amount of energy spent by the amplifier to transmit the bits)
% Data Aggregation Energy %
EDA = 5 * 10 ^ (-9); % units in Joules/bit

for i = 1:n

    SN(i).id = i; % sensor's ID number
    SN(i).Energy = Eo; % nodes energy levels (initially set to be equal to "Eo"
    SN(i).role = 0; % node acts as normal if the value is '0', if elected as a cluster head it gets the value '1' (initially all nodes are normal)
    SN(i).cluster = 0; % the cluster which a node belongs to
    SN(i).condition = 1; % States the current condition of the node. when the node is operational its value is =1 and when dead =0
    SN(i).rop = 0; % number of rounds node was operational
    SN(i).rleft = 0; % rounds left for node to become available for Cluster Head election
    SN(i).dtch = 0; % nodes distance from the cluster head of the cluster in which he belongs
    SN(i).tel = 0; % states how many times the node was elected as a Cluster Head
    SN(i).rn = 0; % round node got elected as cluster head
    SN(i).chid = 0; % node ID of the cluster head which the "i" normal node belongs to
    SN(i).prob = p; % probability with which a node gets to be cluster head

end

CHeads = 0;
CH_list = [];
firstRound = 1;
totalData = 0;
networkStatus = 1;
rad = 200;
dead_nodes = 0;
rounds = 0;

rounds = 0;
totalEnergy = n * Eo;
operating_nodes = n;
transmissions = 0;
temp_val = 0;
flagFirstDead = 0;
stop_flag = 0;

opNodes = [];
dNodes = [];
cNodes = [];
threshData = [];
remainingEnergy = [];
nrg = [];
flagFirstDead = 0;
networkLiveData_2 = [];
abpl_2 = [];
lsp_2 = [];

%-----------Re-initializing values done-----------%

%%%%%% Set-Up Phase %%%%%%
%%%%%%%% PROPOSED W/O DEGREE %%%%%%%%
while (operating_nodes > 0 && stop_flag == 0)

    % Displays Current Round %
    rounds = rounds + 1
    remainingEnergy(rounds) = totalEnergy;

    % Reseting Previous Amount Of Cluster Heads In the Network %
    CHeads = 0;

    % Reseting Previous Amount Of Energy Consumed In the Network on the Previous Round %
    energy = 0;

    % New list of nodes is sorted on basis of energy %
    SN = sortNodes_energy(SN, n);

    CHeads = 0;
    CH_list = [];

    % Cluster Heads Election %
    [SN, CH_list, CHeads] = maxE_minD_selection(SN, n, Ci, Eo);

    if (CHeads ~= 0)
        % Fixing the size of "CL" array %
        CH_list = CH_list(1:CHeads);
        CH_list = arrayfun(@(x) setfield(x, 'degree', 0), CH_list);

        % Grouping the Nodes into Clusters & caclulating the distance between node and cluster head %
        for i = 1:n

            if (SN(i).role == 0) && (SN(i).Energy > 0) && (CHeads > 0) % if node is normal

                for m = 1:CHeads
                    d(m) = sqrt((CH_list(m).x - SN(i).x) ^ 2 + (CH_list(m).y - SN(i).y) ^ 2) * 2;
                    % we calculate the distance 'd' between the sensor node that is
                    % transmitting and the cluster head that is receiving with the following equation+
                    % d=sqrt((x2-x1)^2 + (y2-y1)^2) where x2 and y2 the coordinates of
                    % the cluster head and x1 and y1 the coordinates of the transmitting node
                end

                d = d(1:CHeads); % fixing the size of "d" array
                [M, I] = min(d(:)); % finds the minimum distance of node to CH
                [Row, Col] = ind2sub(size(d), I); % displays the Cluster Number in which this node belongs too
                SN(i).cluster = Col; % assigns node to the cluster
                SN(i).dtch = d(Col); % assigns the distance of node to CH
                SN(i).chid = CH_list(Col).id;
                CH_list(Col).degree = CH_list(Col).degree + 1;

            end

        end

        % printing the degree of each CH %

        % calculating HOPS and Proximity %
        [CH_list, ABPL] = CH_list_with_HOPS(CH_list, CHeads, rad, rounds);
        Network_Longest_Shortest_Path = calculating_lsp(CH_list, CHeads);

        abpl_2(rounds) = ABPL;
        lsp_2(rounds) = Network_Longest_Shortest_Path;

        %%%%%% Steady-State Phase %%%%%%
        % Energy Dissipation for normal nodes %

        for i = 1:n

            if (SN(i).condition == 1) && (SN(i).role == 0)

                if SN(i).Energy > 0
                    ETx = Eelec * k + Eamp * k * SN(i).dtch ^ 2;
                    SN(i).Energy = SN(i).Energy - ETx;
                    energy = energy + ETx;
                    totalEnergy = totalEnergy - ETx;
                    totalData = totalData + k;

                    % Dissipation for cluster head during reception
                    if SN(SN(i).chid).Energy > 0 && SN(SN(i).chid).condition == 1 && SN(SN(i).chid).role == 1
                        ERx = (Eelec + EDA) * k;
                        energy = energy + ERx;
                        totalEnergy = totalEnergy - ERx;

                        SN(SN(i).chid).Energy = SN(SN(i).chid).Energy - ERx;

                        if SN(SN(i).chid).Energy <= 0 % if cluster heads energy depletes with reception
                            SN(SN(i).chid).condition = 0;
                            SN(SN(i).chid).rop = rounds;
                            dead_nodes = dead_nodes + 1;
                            operating_nodes = operating_nodes - 1
                        end

                    end

                end

                if SN(i).Energy < Eo * 0.2 % if nodes energy depletes with transmission
                    dead_nodes = dead_nodes + 1;
                    operating_nodes = operating_nodes - 1
                    SN(i).condition = 0;
                    SN(i).chid = 0;
                    SN(i).rop = rounds;
                end

            end

        end

        % Energy Dissipation for cluster head nodes %

        for i = 1:n

            if (SN(i).condition == 1) && (SN(i).role == 1)

                if SN(i).Energy > 0
                    ETx = (Eelec + EDA) * k + Eamp * k * SN(i).dts ^ 2;
                    SN(i).Energy = SN(i).Energy - ETx;
                    energy = energy + ETx;

                end

                if SN(i).Energy < Eo * 0.2 % if cluster heads energy depletes with transmission
                    dead_nodes = dead_nodes + 1;
                    operating_nodes = operating_nodes - 1
                    SN(i).condition = 0;
                    SN(i).rop = rounds;
                end

            end

        end

    end

    if operating_nodes < n && temp_val == 0
        temp_val = 1;
        flagFirstDead = rounds;
        networkLiveData_2(networkStatus) = rounds;
        networkStatus = networkStatus + 1;
    end

    if operating_nodes <= n * 0.5 && temp_val == 1
        temp_val = 2;
        networkLiveData_2(networkStatus) = rounds;
        networkStatus = networkStatus + 1;
    end

    if flagFirstDead == 0
        transmissions = transmissions + 1;
        nrg(transmissions) = energy;
    end

    if CHeads == 0 || operating_nodes <= 4
        transmissions = transmissions - 1;
        stop_flag = 1;
        CHeads = 0;
        networkLiveData_2(networkStatus) = rounds;
    end

    opNodes(rounds) = operating_nodes;
    dNodes(rounds) = dead_nodes;
    cNodes(rounds) = CHeads;
    threshData(rounds) = totalData;

end

% Plotting Simulation Results "Operating Nodes per Round" %
figure(2)
plot(1:length(opNodes), opNodes(1:length(opNodes)), '-b', 'Linewidth', 2);
hold on;

figure(3)
plot(1:length(dNodes), dNodes(1:length(dNodes)), '-b', 'Linewidth', 2);
hold on;

figure(4)
plot(1:length(cNodes), cNodes(1:length(cNodes)), '-b', 'Linewidth', 2);
hold on;

figure(5)
plot(1:length(threshData), threshData(1:length(threshData)), '-b', 'Linewidth', 2);
hold on;

figure(6)
plot(1:length(nrg), nrg(1:length(nrg)), '-b', 'Linewidth', 2);
hold on;

figure(7)
area(1:length(remainingEnergy), remainingEnergy(1:length(remainingEnergy)), 'FaceColor', 'b', 'FaceAlpha', 0.5);
hold on;

figure(8)
plot(1:length(abpl_2), abpl_2(1:length(abpl_2)), '-b', 'Linewidth', 2);
hold on;

figure(9)
plot(1:length(lsp_2), lsp_2(1:length(lsp_2)), '-b', 'Linewidth', 2);
hold on;

%-----------Re-initializing values-----------%
dead_nodes = 0;
% Coordinates of the Sink (location is predetermined in this simulation) %
sinkx = 990;
sinky = 990;
%%% Energy Values %%%
% Initial Energy of a Node (in Joules) %
Eo = 0.1; % units in Joules
% Energy required to run circuity (both for transmitter and receiver) %
Eelec = 50 * 10 ^ (-9); % units in Joules/bit
ETx = 50 * 10 ^ (-9); % units in Joules/bit
ERx = 50 * 10 ^ (-9); % units in Joules/bit
% Transmit Amplifier Types %
Eamp = 100 * 10 ^ (-12); % units in Joules/bit/m^2 (amount of energy spent by the amplifier to transmit the bits)
% Data Aggregation Energy %
EDA = 5 * 10 ^ (-9); % units in Joules/bit

for i = 1:n

    SN(i).id = i; % sensor's ID number
    SN(i).Energy = Eo; % nodes energy levels (initially set to be equal to "Eo"
    SN(i).role = 0; % node acts as normal if the value is '0', if elected as a cluster head it gets the value '1' (initially all nodes are normal)
    SN(i).cluster = 0; % the cluster which a node belongs to
    SN(i).condition = 1; % States the current condition of the node. when the node is operational its value is =1 and when dead =0
    SN(i).rop = 0; % number of rounds node was operational
    SN(i).rleft = 0; % rounds left for node to become available for Cluster Head election
    SN(i).dtch = 0; % nodes distance from the cluster head of the cluster in which he belongs
    SN(i).tel = 0; % states how many times the node was elected as a Cluster Head
    SN(i).rn = 0; % round node got elected as cluster head
    SN(i).chid = 0; % node ID of the cluster head which the "i" normal node belongs to
    SN(i).prob = p; % probability with which a node gets to be cluster head

end

CHeads = 0;
CH_list = [];
firstRound = 1;
totalData = 0;
networkStatus = 1;
rad = 200;
dead_nodes = 0;
rounds = 0;

rounds = 0;
totalEnergy = n * Eo;
operating_nodes = n;
transmissions = 0;
temp_val = 0;
flagFirstDead = 0;
stop_flag = 0;

opNodes = [];
dNodes = [];
cNodes = [];
threshData = [];
remainingEnergy = [];
nrg = [];
flagFirstDead = 0;
networkLiveData_3 = [];
abpl_3 = [];
lsp_3 = [];

%-----------Re-initializing values done-----------%

%%%%%% Set-Up Phase %%%%%%
%%%%%%% HEED %%%%%%%
while (operating_nodes > 0 && stop_flag == 0)

    % Displays Current Round %
    rounds = rounds + 1
    remainingEnergy(rounds) = totalEnergy;

    % Reseting Previous Amount Of Energy Consumed In the Network on the Previous Round %
    energy = 0;

    % Cluster Heads Election %
    [SN, CH_list, CHeads] = CH_election_HEED(SN, n, p, rounds, CHeads, Eo, CH_list);

    if (CHeads ~= 0)
        % Fixing the size of "CL" array %
        CH_list = CH_list(1:CHeads);
        CH_list = arrayfun(@(x) setfield(x, 'degree', 0), CH_list);

        % Calculating node degree of cluster head based on Distance to cluster members %
        for i = 1:n

            if ((SN(i).role == 0) && (SN(i).Energy > 0)) % if node is normal

                for m = 1:CHeads
                    d(m) = sqrt((CH_list(m).x - SN(i).x) ^ 2 + (CH_list(m).y - SN(i).y) ^ 2);
                    % we calculate the distance 'd' between the sensor node that is
                    % transmitting and the cluster head that is receiving with the following equation+
                    % d=sqrt((x2-x1)^2 + (y2-y1)^2) where x2 and y2 the coordinates of
                    % the cluster head and x1 and y1 the coordinates of the transmitting node
                end

                d = d(1:CHeads); % fixing the size of "d" array
                [M, I] = min(d(:)); % finds the minimum distance of node to CH
                [Row, Col] = ind2sub(size(d), I); % displays the Cluster Number in which this node belongs too
                CH_list(Col).degree = CH_list(Col).degree + 1;

            end

        end

        % adding new field as Cost as a function of node degree to CH_list %
        CH_list = arrayfun(@(x) setfield(x, 'cost', 0), CH_list);

        for i = 1:CHeads

            if (CH_list(i).degree ~= 0)
                CH_list(i).cost = 1 / CH_list(i).degree;
            else
                CH_list(i).cost = 100;
            end

            CH_list(i).degree = 0;

        end

        for i = 1:n

            if ((SN(i).role == 0) && (SN(i).Energy > 0)) % if node is normal

                for m = 1:CHeads
                    c(m) = CH_list(m).cost;
                    % we calculate the distance 'd' between the sensor node that is
                    % transmitting and the cluster head that is receiving with the following equation+
                    % d=sqrt((x2-x1)^2 + (y2-y1)^2) where x2 and y2 the coordinates of
                    % the cluster head and x1 and y1 the coordinates of the transmitting node
                end

                c = c(1:CHeads); % fixing the size of "d" array
                [M, I] = min(c(:)); % finds the minimum distance of node to CH
                [Row, Col] = ind2sub(size(c), I); % displays the Cluster Number in which this node belongs too
                SN(i).cluster = Col; % assigns node to the cluster
                SN(i).dtch = d(Col); % assigns the distance of node to CH
                SN(i).chid = CH_list(Col).id;
                CH_list(Col).degree = CH_list(Col).degree + 1;

            end

        end

        % printing the degree of each CH %

        % calculating HOPS and Proximity %
        [CH_list, ABPL] = CH_list_with_HOPS(CH_list, CHeads, rad, rounds);
        Network_Longest_Shortest_Path = calculating_lsp(CH_list, CHeads);

        abpl_3(rounds) = ABPL;
        lsp_3(rounds) = Network_Longest_Shortest_Path;

        %%%%%% Steady-State Phase %%%%%%
        % Energy Dissipation for normal nodes %

        for i = 1:n

            if (SN(i).condition == 1) && (SN(i).role == 0)

                if SN(i).Energy > 0
                    ETx = Eelec * k + Eamp * k * SN(i).dtch ^ 2;
                    SN(i).Energy = SN(i).Energy - ETx;
                    energy = energy + ETx;
                    totalEnergy = totalEnergy - ETx;
                    totalData = totalData + k;

                    % Dissipation for cluster head during reception
                    if SN(SN(i).chid).Energy > 0 && SN(SN(i).chid).condition == 1 && SN(SN(i).chid).role == 1
                        ERx = (Eelec + EDA) * k;
                        energy = energy + ERx;
                        totalEnergy = totalEnergy - ERx;
                        SN(SN(i).chid).Energy = SN(SN(i).chid).Energy - ERx;

                        if SN(SN(i).chid).Energy <= 0 % if cluster heads energy depletes with reception
                            SN(SN(i).chid).condition = 0;
                            SN(SN(i).chid).rop = rounds;
                            dead_nodes = dead_nodes + 1;
                            operating_nodes = operating_nodes - 1
                        end

                    end

                end

                if SN(i).Energy <= 0 % if nodes energy depletes with transmission
                    dead_nodes = dead_nodes + 1;
                    operating_nodes = operating_nodes - 1
                    SN(i).condition = 0;
                    SN(i).chid = 0;
                    SN(i).rop = rounds;
                end

            end

        end

        % Energy Dissipation for cluster head nodes %

        for i = 1:n

            if (SN(i).condition == 1) && (SN(i).role == 1)

                if SN(i).Energy > 0
                    ETx = (Eelec + EDA) * k + Eamp * k * SN(i).dts ^ 2;
                    SN(i).Energy = SN(i).Energy - ETx;
                    energy = energy + ETx;
                    totalEnergy = totalEnergy - ETx;
                end

                if SN(i).Energy <= 0 % if cluster heads energy depletes with transmission
                    dead_nodes = dead_nodes + 1;
                    operating_nodes = operating_nodes - 1
                    SN(i).condition = 0;
                    SN(i).rop = rounds;
                end

            end

        end

    end

    if operating_nodes < n && temp_val == 0
        temp_val = 1;
        flagFirstDead = rounds;
        networkLiveData_3(networkStatus) = rounds;
        networkStatus = networkStatus + 1;
    end

    if operating_nodes <= n * 0.5 && temp_val == 1
        temp_val = 2;
        networkLiveData_3(networkStatus) = rounds;
        networkStatus = networkStatus + 1;
    end

    if flagFirstDead == 0
        transmissions = transmissions + 1;
        nrg(transmissions) = energy;
    end

    if CHeads == 0 || operating_nodes < 3
        transmissions = transmissions - 1;
        stop_flag = 1;
        CHeads = 0;
        networkLiveData_3(networkStatus) = rounds;
    end

    opNodes(rounds) = operating_nodes;
    dNodes(rounds) = dead_nodes;
    cNodes(rounds) = CHeads;
    threshData(rounds) = totalData;

end

if networkStatus == 1
    networkStatus = networkStatus + 1;
    networkLiveData_3(networkStatus) = rounds;
    networkStatus = networkStatus + 1;
    networkLiveData_3(networkStatus) = rounds;
elseif networkStatus == 2
    networkStatus = networkStatus + 1;
    networkLiveData_3(networkStatus) = rounds;
end

% Plotting Simulation Results "Operating Nodes per Round" %
figure(2)
plot(1:length(opNodes), opNodes(1:length(opNodes)), '-g', 'Linewidth', 2);
hold on;

figure(3)
plot(1:length(dNodes), dNodes(1:length(dNodes)), '-g', 'Linewidth', 2);
hold on;

figure(4)
plot(1:length(cNodes), cNodes(1:length(cNodes)), '-g', 'Linewidth', 2);
hold on;

figure(5)
plot(1:length(threshData), threshData(1:length(threshData)), '-g', 'Linewidth', 2);
hold on;

figure(6)
plot(1:length(nrg), nrg(1:length(nrg)), '-g', 'Linewidth', 2);
hold on;

figure(7)
area(1:length(remainingEnergy), remainingEnergy(1:length(remainingEnergy)), 'FaceColor', 'g', 'FaceAlpha', 0.5);
hold on;

figure(8)
plot(1:length(abpl_3), abpl_3(1:length(abpl_3)), '-g', 'Linewidth', 2);
hold on;

figure(9)
plot(1:length(lsp_3), lsp_3(1:length(lsp_3)), '-g', 'Linewidth', 2);
hold on;

%-----------Re-initializing values-----------%
dead_nodes = 0;
% Coordinates of the Sink (location is predetermined in this simulation) %
sinkx = 990;
sinky = 990;
%%% Energy Values %%%
% Initial Energy of a Node (in Joules) %
Eo = 0.1; % units in Joules
% Energy required to run circuity (both for transmitter and receiver) %
Eelec = 50 * 10 ^ (-9); % units in Joules/bit
ETx = 50 * 10 ^ (-9); % units in Joules/bit
ERx = 50 * 10 ^ (-9); % units in Joules/bit
% Transmit Amplifier Types %
Eamp = 100 * 10 ^ (-12); % units in Joules/bit/m^2 (amount of energy spent by the amplifier to transmit the bits)
% Data Aggregation Energy %
EDA = 5 * 10 ^ (-9); % units in Joules/bit

for i = 1:n

    SN(i).id = i; % sensor's ID number
    SN(i).Energy = Eo; % nodes energy levels (initially set to be equal to "Eo"
    SN(i).role = 0; % node acts as normal if the value is '0', if elected as a cluster head it gets the value '1' (initially all nodes are normal)
    SN(i).cluster = 0; % the cluster which a node belongs to
    SN(i).condition = 1; % States the current condition of the node. when the node is operational its value is =1 and when dead =0
    SN(i).rop = 0; % number of rounds node was operational
    SN(i).rleft = 0; % rounds left for node to become available for Cluster Head election
    SN(i).dtch = 0; % nodes distance from the cluster head of the cluster in which he belongs
    SN(i).tel = 0; % states how many times the node was elected as a Cluster Head
    SN(i).rn = 0; % round node got elected as cluster head
    SN(i).chid = 0; % node ID of the cluster head which the "i" normal node belongs to
    SN(i).prob = p; % probability with which a node gets to be cluster head

end

CHeads = 0;
CLheads = 0;
CH_list = [];
firstRound = 1;
totalData = 0;
networkStatus = 1;
rad = 200;

rounds = 0;
totalEnergy = n * Eo;
operating_nodes = n;
transmissions = 0;
temp_val = 0;
flagFirstDead = 0;
stop_flag = 0;

opNodes = [];
dNodes = [];
cNodes = [];
threshData = [];
remainingEnergy = [];
nrg = [];
flagFirstDead = 0;
networkLiveData_4 = [];
abpl_4 = [];
lsp_4 = [];

%-----------Re-initializing values done-----------%

%%%%%% Set-Up Phase %%%%%%
%%%%%%%%% LEACH-C %%%%%%%%
while (operating_nodes > 0 && stop_flag == 0)

    % Displays Current Round %
    rounds = rounds + 1
    remainingEnergy(rounds) = totalEnergy;

    % Reseting Previous Amount Of Cluster Heads In the Network %

    CLheads = 0;

    % Threshold Value %
    t = (p / (1 - p * (mod(rounds, 1 / p))));

    % Re-election Value %
    tleft = mod(rounds, 1 / p);

    % Reseting Previous Amount Of Energy Consumed In the Network on the Previous Round %
    energy = 0;
    Ecen = 0;

    % average energy calculation %
    for i = 1:n

        if (SN(i).condition == 1)
            Ecen = Ecen + SN(i).Energy;
        end

    end

    Ecen = Ecen / operating_nodes;

    % Cluster Heads Election %
    [SN, CL, CLheads] = CH_election_centralized(SN, n, t, tleft, p, rounds, sinkx, sinky, CLheads, Eo, Ecen);

    % Fixing the size of "CL" array %
    CL = CL(1:CLheads);
    CL = arrayfun(@(x) setfield(x, 'degree', 0), CL);

    % Grouping the Nodes into Clusters & caclulating the distance between node and cluster head %

    for i = 1:n

        if (SN(i).role == 0) && (SN(i).Energy > 0) && (CLheads > 0) % if node is normal

            for m = 1:CLheads
                d(m) = sqrt((CL(m).x - SN(i).x) ^ 2 + (CL(m).y - SN(i).y) ^ 2) * 2;
                % we calculate the distance 'd' between the sensor node that is
                % transmitting and the cluster head that is receiving with the following equation+
                % d=sqrt((x2-x1)^2 + (y2-y1)^2) where x2 and y2 the coordinates of
                % the cluster head and x1 and y1 the coordinates of the transmitting node
            end

            d = d(1:CLheads); % fixing the size of "d" array
            [M, I] = min(d(:)); % finds the minimum distance of node to CH
            [Row, Col] = ind2sub(size(d), I); % displays the Cluster Number in which this node belongs too
            SN(i).cluster = Col; % assigns node to the cluster
            SN(i).dtch = d(Col); % assigns the distance of node to CH
            SN(i).chid = CL(Col).id;
            CL(Col).degree = CL(Col).degree + 1; % increase the degree of the cluster head
        end

    end

    % calculating only if there are CHs in the network %
    if (CLheads ~= 0)

        % calculating HOPS and Proximity %
        [CL, ABPL] = CH_list_with_HOPS(CL, CLheads, rad, rounds);
        Network_Longest_Shortest_Path = calculating_lsp(CL, CLheads);

        abpl_4(rounds) = ABPL;
        lsp_4(rounds) = Network_Longest_Shortest_Path;

    end

    %%%%%% Steady-State Phase %%%%%%
    % Energy Dissipation for normal nodes %

    for i = 1:n

        if (SN(i).condition == 1) && (SN(i).role == 0) && (CLheads > 0)

            if SN(i).Energy > 0
                ETx = Eelec * k + Eamp * k * SN(i).dtch ^ 2;
                SN(i).Energy = SN(i).Energy - ETx;
                energy = energy + ETx;
                totalEnergy = totalEnergy - ETx;
                totalData = totalData + k;

                % Dissipation for cluster head during reception
                if SN(SN(i).chid).Energy > 0 && SN(SN(i).chid).condition == 1 && SN(SN(i).chid).role == 1
                    ERx = (Eelec + EDA) * k;
                    energy = energy + ERx;
                    totalEnergy = totalEnergy - ERx;
                    SN(SN(i).chid).Energy = SN(SN(i).chid).Energy - ERx;

                    if SN(SN(i).chid).Energy <= Eo * 0.2 % if cluster heads energy depletes with reception
                        SN(SN(i).chid).condition = 0;
                        SN(SN(i).chid).rop = rounds;
                        dead_nodes = dead_nodes +1;
                        operating_nodes = operating_nodes - 1
                    end

                end

            end

            if SN(i).Energy <= Eo * 0.2 % if nodes energy depletes with transmission
                dead_nodes = dead_nodes + 1;
                operating_nodes = operating_nodes - 1
                SN(i).condition = 0;
                SN(i).chid = 0;
                SN(i).rop = rounds;
            end

        end

    end

    % Energy Dissipation for cluster head nodes %

    for i = 1:n

        if (SN(i).condition == 1) && (SN(i).role == 1)

            if SN(i).Energy > 0
                ETx = (Eelec + EDA) * k + Eamp * k * SN(i).dts ^ 2;
                SN(i).Energy = SN(i).Energy - ETx;
                energy = energy + ETx;
                totalEnergy = totalEnergy - ETx;
            end

            if SN(i).Energy <= Eo * 0.2 % if cluster heads energy depletes with transmission
                dead_nodes = dead_nodes +1;
                operating_nodes = operating_nodes - 1
                SN(i).condition = 0;
                SN(i).rop = rounds;
            end

        end

    end

    if operating_nodes < n && temp_val == 0
        temp_val = 1;
        flagFirstDead = rounds;
        networkLiveData_4(networkStatus) = rounds;
        networkStatus = networkStatus + 1;
    end

    if operating_nodes <= n * 0.5 && temp_val == 1
        temp_val = 2;
        networkLiveData_4(networkStatus) = rounds;
        networkStatus = networkStatus + 1;
    end

    if flagFirstDead == 0
        transmissions = transmissions + 1;
        nrg(transmissions) = energy;
    end

    cnt = 0;

    for i = 1:n

        if (SN(i).Energy <= Eo * 0.2)
            cnt = cnt + 1;
        end

    end

    if cnt == n || operating_nodes < 3
        networkLiveData_4(networkStatus) = rounds;
        stop_flag = 1;
    end

    opNodes(rounds) = operating_nodes;
    dNodes(rounds) = dead_nodes;
    cNodes(rounds) = CLheads;
    threshData(rounds) = totalData;

end

if networkStatus == 1
    networkStatus = networkStatus + 1;
    networkLiveData_4(networkStatus) = rounds;
    networkStatus = networkStatus + 1;
    networkLiveData_4(networkStatus) = rounds;
elseif networkStatus == 2
    networkStatus = networkStatus + 1;
    networkLiveData_4(networkStatus) = rounds;
end

% Plotting Simulation Results "Operating Nodes per Round" %
figure(2)
plot(1:length(opNodes), opNodes(1:length(opNodes)), '-c', 'Linewidth', 2);
hold on;

figure(3)
plot(1:length(dNodes), dNodes(1:length(dNodes)), '-c', 'Linewidth', 2);
hold on;

figure(4)
plot(1:length(cNodes), cNodes(1:length(cNodes)), '-c', 'Linewidth', 2);
hold on;

figure(5)
plot(1:length(threshData), threshData(1:length(threshData)), '-c', 'Linewidth', 2);
hold on;

figure(6)
plot(1:length(nrg), nrg(1:length(nrg)), '-c', 'Linewidth', 2);
hold on;

figure(7)
area(1:length(remainingEnergy), remainingEnergy(1:length(remainingEnergy)), 'FaceColor', 'c', 'FaceAlpha', 0.5);
hold on;

figure(8)
plot(1:length(abpl_4), abpl_4(1:length(abpl_4)), '-c', 'Linewidth', 2);
hold on;

figure(9)
plot(1:length(lsp_4), lsp_4(1:length(lsp_4)), '-c', 'Linewidth', 2);
hold on;

%-----------Re-initializing values-----------%
dead_nodes = 0;
% Coordinates of the Sink (location is predetermined in this simulation) %
sinkx = 990;
sinky = 990;
%%% Energy Values %%%
% Initial Energy of a Node (in Joules) %
Eo = 0.1; % units in Joules
% Energy required to run circuity (both for transmitter and receiver) %
Eelec = 50 * 10 ^ (-9); % units in Joules/bit
ETx = 50 * 10 ^ (-9); % units in Joules/bit
ERx = 50 * 10 ^ (-9); % units in Joules/bit
% Transmit Amplifier Types %
Eamp = 100 * 10 ^ (-12); % units in Joules/bit/m^2 (amount of energy spent by the amplifier to transmit the bits)
% Data Aggregation Energy %
EDA = 5 * 10 ^ (-9); % units in Joules/bit

for i = 1:n

    SN(i).id = i; % sensor's ID number
    SN(i).Energy = Eo; % nodes energy levels (initially set to be equal to "Eo"
    SN(i).role = 0; % node acts as normal if the value is '0', if elected as a cluster head it gets the value '1' (initially all nodes are normal)
    SN(i).cluster = 0; % the cluster which a node belongs to
    SN(i).condition = 1; % States the current condition of the node. when the node is operational its value is =1 and when dead =0
    SN(i).rop = 0; % number of rounds node was operational
    SN(i).rleft = 0; % rounds left for node to become available for Cluster Head election
    SN(i).dtch = 0; % nodes distance from the cluster head of the cluster in which he belongs
    SN(i).tel = 0; % states how many times the node was elected as a Cluster Head
    SN(i).rn = 0; % round node got elected as cluster head
    SN(i).chid = 0; % node ID of the cluster head which the "i" normal node belongs to
    SN(i).prob = p; % probability with which a node gets to be cluster head

end

CHeads = 0;
CLheads = 0;
CH_list = [];
firstRound = 1;
totalData = 0;
networkStatus = 1;
rad = 200;

rounds = 0;
totalEnergy = n * Eo;
operating_nodes = n;
transmissions = 0;
temp_val = 0;
flagFirstDead = 0;
stop_flag = 0;

opNodes = [];
dNodes = [];
cNodes = [];
threshData = [];
remainingEnergy = [];
nrg = [];
flagFirstDead = 0;
networkLiveData_5 = [];
abpl_5 = [];
lsp_5 = [];

%-----------Re-initializing values done-----------%

%%%%%% Set-Up Phase %%%%%%
%%%%%%%%%% LEACH %%%%%%%%%
while (operating_nodes > 0 && stop_flag == 0)

    % Displays Current Round %
    rounds = rounds + 1
    remainingEnergy(rounds) = totalEnergy;

    % Threshold Value %
    t = (p / (1 - p * (mod(rounds, 1 / p))));

    % Re-election Value %
    tleft = mod(rounds, 1 / p);

    % Reseting Previous Amount Of Cluster Heads In the Network %
    CLheads = 0;

    % Reseting Previous Amount Of Energy Consumed In the Network on the Previous Round %
    energy = 0;

    % Cluster Heads Election %
    [SN, CL, CLheads] = CH_election(SN, n, t, tleft, p, rounds, sinkx, sinky, CLheads, Eo);

    % Fixing the size of "CL" array %
    CL = CL(1:CLheads);
    CL = arrayfun(@(x) setfield(x, 'degree', 0), CL);

    % Grouping the Nodes into Clusters & caclulating the distance between node and cluster head %

    for i = 1:n

        if (SN(i).role == 0) && (SN(i).Energy > 0) && (CLheads > 0) % if node is normal

            for m = 1:CLheads
                d(m) = sqrt((CL(m).x - SN(i).x) ^ 2 + (CL(m).y - SN(i).y) ^ 2) * 2;
                % we calculate the distance 'd' between the sensor node that is
                % transmitting and the cluster head that is receiving with the following equation+
                % d=sqrt((x2-x1)^2 + (y2-y1)^2) where x2 and y2 the coordinates of
                % the cluster head and x1 and y1 the coordinates of the transmitting node
            end

            d = d(1:CLheads); % fixing the size of "d" array
            [M, I] = min(d(:)); % finds the minimum distance of node to CH
            [Row, Col] = ind2sub(size(d), I); % displays the Cluster Number in which this node belongs too
            SN(i).cluster = Col; % assigns node to the cluster
            SN(i).dtch = d(Col); % assigns the distance of node to CH
            SN(i).chid = CL(Col).id;
            CL(Col).degree = CL(Col).degree + 1; % increase the degree of the cluster head
        end

    end

    if (CLheads ~= 0)
        % printing the degree of each CH %

        % calculating HOPS and Proximity %
        [CL, ABPL] = CH_list_with_HOPS(CL, CLheads, rad, rounds);
        Network_Longest_Shortest_Path = calculating_lsp(CL, CLheads);

        abpl_5(rounds) = ABPL;
        lsp_5(rounds) = Network_Longest_Shortest_Path;

    end

    %%%%%% Steady-State Phase %%%%%%

    % Energy Dissipation for normal nodes %

    for i = 1:n

        if (SN(i).condition == 1) && (SN(i).role == 0) && (CLheads > 0)

            if SN(i).Energy > 0
                ETx = Eelec * k + Eamp * k * SN(i).dtch ^ 2;
                SN(i).Energy = SN(i).Energy - ETx;
                energy = energy + ETx;
                totalEnergy = totalEnergy - ETx;
                totalData = totalData + k;

                % Dissipation for cluster head during reception
                if SN(SN(i).chid).Energy > 0 && SN(SN(i).chid).condition == 1 && SN(SN(i).chid).role == 1
                    ERx = (Eelec + EDA) * k;
                    energy = energy + ERx;
                    totalEnergy = totalEnergy - ERx;
                    SN(SN(i).chid).Energy = SN(SN(i).chid).Energy - ERx;

                    if SN(SN(i).chid).Energy <= Eo * 0.2 % if cluster heads energy depletes with reception
                        SN(SN(i).chid).condition = 0;
                        SN(SN(i).chid).rop = rounds;
                        dead_nodes = dead_nodes + 1;
                        operating_nodes = operating_nodes - 1
                    end

                end

            end

            if SN(i).Energy <= Eo * 0.2 % if nodes energy depletes with transmission
                dead_nodes = dead_nodes + 1;
                operating_nodes = operating_nodes - 1
                SN(i).condition = 0;
                SN(i).chid = 0;
                SN(i).rop = rounds;
            end

        end

    end

    % Energy Dissipation for cluster head nodes %

    for i = 1:n

        if (SN(i).condition == 1) && (SN(i).role == 1)

            if SN(i).Energy > 0
                ETx = (Eelec + EDA) * k + Eamp * k * SN(i).dts ^ 2;
                SN(i).Energy = SN(i).Energy - ETx;
                energy = energy + ETx;
                totalEnergy = totalEnergy - ETx;
            end

            if SN(i).Energy <= Eo * 0.2 % if cluster heads energy depletes with transmission
                dead_nodes = dead_nodes +1;
                operating_nodes = operating_nodes - 1
                SN(i).condition = 0;
                SN(i).rop = rounds;
            end

        end

    end

    if operating_nodes < n && temp_val == 0
        temp_val = 1;
        flagFirstDead = rounds;
        networkLiveData_5(networkStatus) = rounds;
        networkStatus = networkStatus + 1;
    end

    if operating_nodes <= n * 0.5 && temp_val == 1
        temp_val = 2;
        networkLiveData_5(networkStatus) = rounds;
        networkStatus = networkStatus + 1;
    end

    if flagFirstDead == 0
        transmissions = transmissions + 1;
        nrg(transmissions) = energy;
    end

    cnt = 0;

    for i = 1:n

        if (SN(i).Energy <= Eo * 0.2)
            cnt = cnt + 1;
        end

    end

    if cnt == n || operating_nodes < 3
        networkLiveData_5(networkStatus) = rounds;
        stop_flag = 1;
    end

    opNodes(rounds) = operating_nodes;
    dNodes(rounds) = dead_nodes;
    cNodes(rounds) = CLheads;
    threshData(rounds) = totalData;

end

if networkStatus == 1
    networkStatus = networkStatus + 1;
    networkLiveData_5(networkStatus) = rounds;
    networkStatus = networkStatus + 1;
    networkLiveData_5(networkStatus) = rounds;
elseif networkStatus == 2
    networkStatus = networkStatus + 1;
    networkLiveData_5(networkStatus) = rounds;
end

% Plotting Simulation Results "Operating Nodes per Round" %
figure(2)
plot(1:length(opNodes), opNodes(1:length(opNodes)), '-k', 'Linewidth', 2);
l = {'PROPOSED WITH DEGREE', 'PROPOSED W/O DEGREE', 'HEED', 'LEACH-C', 'LEACH'};
legend(l, 'Location', 'NorthEast');

figure(3)
plot(1:length(dNodes), dNodes(1:length(dNodes)), '-k', 'Linewidth', 2);
l = {'PROPOSED WITH DEGREE', 'PROPOSED W/O DEGREE', 'HEED', 'LEACH-C', 'LEACH'};
legend(l, 'Location', 'SouthEast');

figure(4)
plot(1:length(cNodes), cNodes(1:length(cNodes)), '-k', 'Linewidth', 2);
l = {'PROPOSED WITH DEGREE', 'PROPOSED W/O DEGREE', 'HEED', 'LEACH-C', 'LEACH'};
legend(l, 'Location', 'NorthEast');

figure(5)
plot(1:length(threshData), threshData(1:length(threshData)), '-k', 'Linewidth', 2);
l = {'PROPOSED WITH DEGREE', 'PROPOSED W/O DEGREE', 'HEED', 'LEACH-C', 'LEACH'};
legend(l, 'Location', 'Best');

figure(6)
plot(1:length(nrg), nrg(1:length(nrg)), '-k', 'Linewidth', 2);
% obj = findobj('Parent', figure(6), 'Type', 'Line', 'Color', 'r');
% uistack(obj, 'top');
l = {'PROPOSED WITH DEGREE', 'PROPOSED W/O DEGREE', 'HEED', 'LEACH-C', 'LEACH'};
legend(l, 'Location', 'NorthEast');

figure(7)
area(1:length(remainingEnergy), remainingEnergy(1:length(remainingEnergy)), 'FaceColor', 'k', 'FaceAlpha', 0.5);
l = {'PROPOSED WITH DEGREE', 'PROPOSED W/O DEGREE', 'HEED', 'LEACH-C', 'LEACH'};
legend(l, 'Location', 'NorthEast');

figure(8)
plot(1:length(abpl_5), abpl_5(1:length(abpl_5)), '-k', 'Linewidth', 2);
l = {'PROPOSED WITH DEGREE', 'PROPOSED W/O DEGREE', 'HEED', 'LEACH-C', 'LEACH'};
legend(l, 'Location', 'NorthEast');

figure(9)
plot(1:length(lsp_5), lsp_5(1:length(lsp_5)), '-k', 'Linewidth', 2);
l = {'PROPOSED WITH DEGREE', 'PROPOSED W/O DEGREE', 'HEED', 'LEACH-C', 'LEACH'};
legend(l, 'Location', 'NorthEast');

figure(10)
x = categorical({'First Dead Node', 'Half Network Dead', 'Full Network Dead'});
x = reordercats(x, {'First Dead Node', 'Half Network Dead', 'Full Network Dead'});
y = [networkLiveData_1; networkLiveData_2; networkLiveData_3; networkLiveData_4; networkLiveData_5];
g = bar(x, y, 'grouped');

set(g(1), 'FaceColor', 'r')
set(g(2), 'FaceColor', 'b')
set(g(3), 'FaceColor', 'g')
set(g(4), 'FaceColor', 'c')
set(g(5), 'FaceColor', 'k')

l = {'PROPOSED WITH DEGREE', 'PROPOSED W/O DEGREE', 'HEED', 'LEACH-C', 'LEACH'};
legend(l, 'Location', 'NorthWest');

title ({'COMPARISION', 'Network Status'; })
xlabel 'Network Dead';
ylabel '# Rounds';

%% ALL FUNCTIONS %%
% sorting nodes based on energy
function S = sortNodes_energy(S, Ni)

    for i = 1:Ni

        S(i).role = 0;

        for j = 1:Ni

            if (S(j).condition == 0)
                break;
            end

            if S(i).Energy > S(j).Energy
                temp = S(i);
                S(i) = S(j);
                S(j) = temp;
            end

        end

    end

end

% sorting nodes based on probability
function S = sortNodes_prob(S, Ni)

    for i = 1:Ni

        S(i).role = 0;

        for j = 1:Ni

            if (S(j).condition == 0)
                break;
            end

            if S(i).prob > S(j).prob
                temp = S(i);
                S(i) = S(j);
                S(j) = temp;
            end

        end

    end

end

% algorithm for PROPOSED WITH DEGREE
function [S, CH_list, CHeads, firstRound] = maxE_minD_selection_new(S, Ni, Ci, Eo, CH_list, CHeads, firstRound, operating_nodes)

    if (firstRound == 1)
        CH_list = [];
        CHeads = 0;

        for k = 1:Ci
            S(k).cluster = 0;
            S(k).role = 0;
            S(k).chid = 0;
            index = -1;
            count = 0;
            minD = 10 ^ 10;
            maxE = -1;

            % selecting cluster heads based on max energy and min distance
            for i = 1:Ni

                if (S(i).role == 1 || S(i).condition == 0)
                    continue;
                end

                distance = 0;

                for j = 1:Ni

                    if (S(j).condition == 0)
                        continue;
                    end

                    if (i ~= j)
                        distance = distance + sqrt((S(i).x - S(j).x) ^ 2 + (S(i).y - S(j).y) ^ 2);
                        count = count + 1;
                    end

                end

                if ((distance / count) <= minD && (S(i).Energy >= maxE) && (S(i).Energy >= Eo * 0.2))
                    minD = distance;
                    maxE = S(i).Energy;
                    index = i;
                end

            end

            if (index ~= -1)
                S(index).role = 1; % 1 = cluster head
                S(index).tel = S(index).tel + 1; % total number of times it became cluster head
                CHeads = CHeads + 1;
                S(index).cluster = CHeads; % cluster number this node belongs to
                CH_list(CHeads).id = S(index).id;
                CH_list(CHeads).x = S(index).x;
                CH_list(CHeads).y = S(index).y;
                CH_list(CHeads).dts = S(index).dts;
            end

        end

        firstRound
        firstRound = firstRound + 1;

    else
        temp_CH_list = [];
        temp_CHeads = 0;

        for k = 1:CHeads

            if (S(CH_list(k).id).Energy < Eo * 0.2 || CH_list(k).degree < operating_nodes * 0.03)

                S(CH_list(k).id).role = 0;
                index = -1;
                count = 0;
                minD = 10 ^ 10;
                maxE = -1;

                for i = 1:Ni

                    if (S(i).role == 1 || S(i).condition == 0 || CH_list(k).id == i)
                        continue;
                    end

                    distance = 0;

                    for j = 1:Ni

                        if (S(j).condition == 0)
                            continue;
                        end

                        if (i ~= j)
                            distance = distance + sqrt((S(i).x - S(j).x) ^ 2 + (S(i).y - S(j).y) ^ 2);
                            count = count + 1;
                        end

                    end

                    if ((distance / count) <= minD && (S(i).Energy >= maxE) && (S(i).Energy >= Eo * 0.2))
                        minD = distance;
                        maxE = S(i).Energy;
                        index = i;
                    end

                end

                if (index ~= -1)
                    S(index).role = 1; % 1 = cluster head
                    S(index).tel = S(index).tel + 1; % total number of times it became cluster head
                    S(index).cluster = k; % cluster number this node belongs to
                    CH_list(k).id = S(index).id;
                    CH_list(k).x = S(index).x;
                    CH_list(k).y = S(index).y;
                    CH_list(k).dts = S(index).dts;

                    temp_CHeads = temp_CHeads + 1;
                    temp_CH_list(temp_CHeads).id = CH_list(k).id;
                    temp_CH_list(temp_CHeads).x = CH_list(k).x;
                    temp_CH_list(temp_CHeads).y = CH_list(k).y;
                    temp_CH_list(temp_CHeads).dts = CH_list(k).dts;
                end

            else
                temp_CHeads = temp_CHeads + 1;
                temp_CH_list(temp_CHeads).id = CH_list(k).id;
                temp_CH_list(temp_CHeads).x = CH_list(k).x;
                temp_CH_list(temp_CHeads).y = CH_list(k).y;
                temp_CH_list(temp_CHeads).dts = CH_list(k).dts;
            end

        end

        CH_list = temp_CH_list;
        CHeads = temp_CHeads;
    end

end

% algorithm for PROPOSED W/O DEGREE
function [S, CH_list, CHeads] = maxE_minD_selection(S, Ni, Ci, Eo)
    CH_list = [];
    CHeads = 0;

    for k = 1:Ci
        S(k).cluster = 0;
        S(k).role = 0;
        S(k).chid = 0;
        index = -1;
        count = 0;
        minD = 10 ^ 10;
        maxE = -1;

        % selecting cluster heads based on max energy and min distance
        for i = 1:Ni

            if (S(i).role == 1 || S(i).condition == 0)
                continue;
            end

            distance = 0;

            for j = 1:Ni

                if (S(j).condition == 0)
                    continue;
                end

                if (i ~= j)
                    distance = distance + sqrt((S(i).x - S(j).x) ^ 2 + (S(i).y - S(j).y) ^ 2);
                    count = count + 1;
                end

            end

            if ((distance / count) <= minD && (S(i).Energy >= maxE) && (S(i).Energy >= Eo * 0.2))
                minD = distance;
                maxE = S(i).Energy;
                index = i;
            end

        end

        if (index ~= -1)
            S(index).role = 1; % 1 = cluster head
            S(i).tel = S(i).tel + 1; % total number of times it became cluster head
            CHeads = CHeads + 1;
            S(i).cluster = CHeads; % cluster number this node belongs to
            CH_list(CHeads).id = S(index).id;
            CH_list(CHeads).x = S(index).x;
            CH_list(CHeads).y = S(index).y;
            CH_list(CHeads).dts = S(index).dts;
        end

    end

end

% algorithm for HEED
function [SN, CH_list, CHeads] = CH_election_HEED(SN, n, p, rounds, CHeads, Eo, CH_list)

    CHeads = 0;

    for i = 1:n
        mult_factor = SN(i).Energy / Eo;
        SN(i).prob = SN(i).prob * mult_factor;
    end

    SN = sortNodes_prob(SN, n);

    for i = 1:n
        SN(i).cluster = 0; % reseting cluster in which the node belongs to
        SN(i).role = 0; % reseting node role
        SN(i).chid = 0; % reseting cluster head id

        if (SN(i).condition == 1 && SN(i).Energy >= Eo * 0.2) % if node is alive and operational

            if (SN(i).prob == 1) % case when its sure to become a cluster head
                SN(i).role = 1; % assigns the node role of acluster head
                SN(i).rn = rounds; % Assigns the round that the cluster head was elected to the data table
                SN(i).tel = SN(i).tel + 1; % Assigns the number of times the cluster head was elected to the data table
                CHeads = CHeads + 1; % sum of cluster heads that have been elected
                SN(i).cluster = CHeads; % cluster of which the node got elected to be cluster head
                CH_list(CHeads).x = SN(i).x; % X-axis coordinates of elected cluster head
                CH_list(CHeads).y = SN(i).y; % Y-axis coordinates of elected cluster head
                CH_list(CHeads).id = i; % Assigns the node ID of the newly elected cluster head to an array
                CH_list(CHeads).dts = SN(i).dts;

            else
                rand = randi([0, 1], 1, 1); % generates a random number between 0 and 1

                if (rand < SN(i).prob) % if the random number is less than the probability of becoming a cluster head
                    SN(i).role = 1; % assigns the node role of acluster head
                    SN(i).rn = rounds; % Assigns the round that the cluster head was elected to the data table
                    SN(i).tel = SN(i).tel + 1; % Assigns the number of times the cluster head was elected to the data table
                    SN(i).prob = min(SN(i).prob * 2, 1); % Assigns the probability of becoming a cluster head to the data table
                    CHeads = CHeads + 1; % sum of cluster heads that have been elected
                    SN(i).cluster = CHeads; % cluster of which the node got elected to be cluster head
                    CH_list(CHeads).x = SN(i).x; % X-axis coordinates of elected cluster head
                    CH_list(CHeads).y = SN(i).y; % Y-axis coordinates of elected cluster head
                    CH_list(CHeads).id = i; % Assigns the node ID of the newly elected cluster head to an array
                    CH_list(CHeads).dts = SN(i).dts;
                end

            end

        end

        if (CHeads == n * p)
            break;
        end

    end

end

% algorithm for LEACH-C
function [SN, CL, CLheads] = CH_election_centralized(SN, n, t, tleft, p, rounds, sinkx, sinky, CLheads, Eo, Ecen)
    CL = [];

    for i = 1:n
        SN(i).cluster = 0; % reseting cluster in which the node belongs to
        SN(i).role = 0; % reseting node role
        SN(i).chid = 0; % reseting cluster head id

        if SN(i).rleft > 0
            SN(i).rleft = SN(i).rleft - 1;
        end

        if (SN(i).Energy > 0) && (SN(i).rleft == 0)
            generate = rand;

            if (generate < t && SN(i).Energy >= Eo * 0.2 && SN(i).Energy >= Ecen)
                SN(i).role = 1; % assigns the node role of acluster head
                SN(i).rn = rounds; % Assigns the round that the cluster head was elected to the data table
                SN(i).tel = SN(i).tel + 1;
                SN(i).rleft = 1 / p - tleft; % rounds for which the node will be unable to become a CH
                SN(i).dts = sqrt((sinkx - SN(i).x) ^ 2 + (sinky - SN(i).y) ^ 2); % calculates the distance between the sink and the cluster hea
                CLheads = CLheads + 1; % sum of cluster heads that have been elected
                SN(i).cluster = CLheads; % cluster of which the node got elected to be cluster head
                CL(CLheads).x = SN(i).x; % X-axis coordinates of elected cluster head
                CL(CLheads).y = SN(i).y; % Y-axis coordinates of elected cluster head
                CL(CLheads).id = i; % Assigns the node ID of the newly elected cluster head to an array
                CL(CLheads).dts = SN(i).dts; % Assigns the distance between the sink and the cluster head to an array
            end

        end

        if CLheads == n * p
            break;
        end

    end

end

% algorithm for LEACH
function [SN, CL, CLheads] = CH_election(SN, n, t, tleft, p, rounds, sinkx, sinky, CLheads, Eo)
    CL = [];

    for i = 1:n
        SN(i).cluster = 0; % reseting cluster in which the node belongs to
        SN(i).role = 0; % reseting node role
        SN(i).chid = 0; % reseting cluster head id

        if SN(i).rleft > 0
            SN(i).rleft = SN(i).rleft - 1;
        end

        if (SN(i).Energy > 0) && (SN(i).rleft == 0)
            generate = rand;

            if (generate < t && SN(i).Energy >= Eo * 0.2)
                SN(i).role = 1; % assigns the node role of a cluster head
                SN(i).rn = rounds; % Assigns the round that the cluster head was elected to the data table
                SN(i).tel = SN(i).tel + 1;
                SN(i).rleft = 1 / p - tleft; % rounds for which the node will be unable to become a CH
                SN(i).dts = sqrt((sinkx - SN(i).x) ^ 2 + (sinky - SN(i).y) ^ 2); % calculates the distance between the sink and the cluster hea
                CLheads = CLheads + 1; % sum of cluster heads that have been elected
                SN(i).cluster = CLheads; % cluster of which the node got elected to be cluster head
                CL(CLheads).x = SN(i).x; % X-axis coordinates of elected cluster head
                CL(CLheads).y = SN(i).y; % Y-axis coordinates of elected cluster head
                CL(CLheads).id = i; % Assigns the node ID of the newly elected cluster head to an array
                CL(CLheads).dts = SN(i).dts; % Assigns the distance between the sink and the cluster head to an array
            end

        end

        if CLheads == n * p
            break;
        end

    end

end

%% GENERAL ALGORITHMS FOR ALL PROTOCOLS %%
% sorting nodes based on proximity
function proximity_list = sortNodes_dts(CH_list, proximity_list)

    if length(proximity_list) == 1
        return;
    end

    for i = 1:length(proximity_list)

        if proximity_list(i) == 0
            continue;
        end

        for j = 1:length(proximity_list)

            if proximity_list(j) == 0
                continue;
            end

            if CH_list(proximity_list(i)).dts > CH_list(proximity_list(j)).dts
                temp = proximity_list(i);
                proximity_list(i) = proximity_list(j);
                proximity_list(j) = temp;
            end

        end

    end

end

% index finding function
function bool = is_found(proximity, index)

    bool = 0;

    for i = 1:length(proximity)

        if (proximity(i) == index)
            bool = 1;
            break;
        end

    end

end

% function to calculate proximity for CHs
function CH_list = CH_list_with_proximity(CH_list, CHeads, rad, rounds)

    CH_list = arrayfun(@(x) setfield(x, 'proximity', []), CH_list);

    % assinging the index of sink as 0 to the proximity list of each CH if it is within the communication range
    for i = 1:CHeads

        if (CH_list(i).dts <= rad)
            CH_list(i).proximity = [CH_list(i).proximity, 0];
        end

    end

    % assinging the index of CHs as their proximity list if they are within the communication range
    for i = 1:CHeads

        for j = 1:CHeads

            if (i ~= j)

                if (sqrt((CH_list(i).x - CH_list(j).x) ^ 2 + (CH_list(i).y - CH_list(j).y) ^ 2) <= rad)
                    CH_list(i).proximity = [CH_list(i).proximity, j];
                end

            end

        end

    end

    for i = 1:CHeads
        CH_list(i).proximity = sortNodes_dts(CH_list, CH_list(i).proximity);
    end

end

% function to calculate the number of hops for each CH
function [hops, indices] = calculate_hops(CH_list, index, indices, hops, rounds)

    if (length(CH_list(index).proximity) ~= 0 && CH_list(index).proximity(1) == 0)
        hops = 1;
    else
        nextIndex = 0;

        for i = 1:length(CH_list(index).proximity)

            for j = 1:length(indices)

                if CH_list(index).proximity(i) == indices(j)
                    nextIndex = 0;
                    break;
                else
                    nextIndex = CH_list(index).proximity(i);
                end

            end

            if nextIndex ~= 0
                break;
            end

        end

        if (nextIndex == 0)

        else

            indices = [indices, nextIndex];
            [hops, indices] = calculate_hops(CH_list, nextIndex, indices, hops, rounds);
            hops = hops + 1;
        end

    end

end

% driver function to calculate the number of hops for each CH
function [CH_list, ABPL] = CH_list_with_HOPS(CH_list, CHeads, rad, rounds)

    CH_list = CH_list_with_proximity(CH_list, CHeads, rad, rounds);
    CH_list = arrayfun(@(x) setfield(x, 'hops', 0), CH_list);

    totalHops = 0;

    % calculating the hops of each CH from the sink
    for i = 1:CHeads

        % if the CH is directly within the communication range of the sink
        if (is_found(CH_list(i).proximity, 0) == 1)
            CH_list(i).hops = 1;
            CH_list(i).lsp_path = [0];

        else
            indices = [];
            indices = [indices, i];
            hops = 0;
            [hops, indices] = calculate_hops(CH_list, i, indices, hops, rounds);

            CH_list(i).hops = hops;
            CH_list(i).lsp_path = indices;
        end

        totalHops = totalHops + CH_list(i).hops;

    end

    ABPL = totalHops / (CHeads * (CHeads + 1) / 2);

end

% calculating the Longest Shortest Path for each CH
function lsp = calculating_lsp(CH_list, CHeads)
    lsp = -1;

    for i = 1:CHeads
        CH_list(i).lsp = 0;

        if CH_list(i).lsp_path(1) == 0
            CH_list(i).lsp = CH_list(i).dts;
        else

            for j = 1:length(CH_list(i).lsp_path)

                if CH_list(i).lsp_path(j) == 0
                    continue;
                else
                    CH_list(i).lsp = CH_list(i).lsp + (sqrt((CH_list(i).x - CH_list(CH_list(i).lsp_path(j)).x) ^ 2 + (CH_list(i).y - CH_list(CH_list(i).lsp_path(j)).y) ^ 2));

                end

            end

        end

    end

    for i = 1:CHeads
        lsp = max(lsp, CH_list(i).lsp);
    end

end
