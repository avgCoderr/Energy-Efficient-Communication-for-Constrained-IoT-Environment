%Rishabh Barnwal%
%2020A7PS1677P%
%2023%
close all;
clear;
clc;
%%%%%%%%%%%%%%%%%%%% Network Establishment Parameters %%%%%%%%%%%%%%%%%%%%
%%% plot of Operation %%%
% Field Dimensions in meters %
xm = 100;
ym = 100;
x = 0; % added for better display results of the plot
y = 0; % added for better display results of the plot
% Number of Nodes in the field %
n = 100;
% Number of Dead Nodes in the beggining %
dead_nodes = 0;
% Coordinates of the Sink (location is predetermined in this simulation) %
sinkx = 75;
sinky = 75;
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
%%%%%%%%%%%%%%%%%%%%%%%%%%% End of Parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Creation of the Wireless Sensor Network %%%
% Plotting the WSN %
for i = 1:n

    SN(i).id = i; % sensor's ID number
    SN(i).x = rand(1, 1) * xm; % X-axis coordinates of sensor node
    SN(i).y = rand(1, 1) * ym; % Y-axis coordinates of sensor node
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

    hold on;
    figure(1)
    plot(x, y, xm, ym, SN(i).x, SN(i).y, 'ob', sinkx, sinky, '*r');
    title 'Wireless Sensor Network';
    xlabel 'X-coordinate';
    ylabel 'Y-coordinate';

end

CHeads = 0;
CH_list = [];
firstRound = 1;
totalData = 0;
networkStatus = 1;
rad = 55;

fid = fopen('node_degree.txt', 'w');
fclose(fid);

fid = fopen('hops.txt', 'w');
fclose(fid);

fid = fopen('proximity.txt', 'w');
fclose(fid);

fid = fopen('debug.txt', 'w');
fclose(fid);

%%%%%% Set-Up Phase %%%%%%

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
        fid = fopen('node_degree.txt', 'a');
        fprintf(fid, '\nRound %d\n', rounds);

        for i = 1:CHeads

            if CH_list(i).degree < operating_nodes * 0.03
                fprintf(fid, 'Degree of CH(%d) = %d (node degree below threshold)\n', i, CH_list(i).degree);
            else
                fprintf(fid, 'Degree of CH(%d) = %d\n', i, CH_list(i).degree);
            end

        end

        fclose(fid);

        % printing the hops of each CH %
        CH_list = CH_list_with_HOPS(CH_list, CHeads, rad, rounds);

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
        flagFirstDead = rounds
        networkLiveData(networkStatus) = rounds;
        networkStatus = networkStatus + 1;
    end

    if operating_nodes <= n * 0.5 && temp_val == 1
        temp_val = 2;
        networkLiveData(networkStatus) = rounds;
        networkStatus = networkStatus + 1;
    end

    transmissions = transmissions + 1;

    if CHeads == 0
        transmissions = transmissions - 1;
        stop_flag = 1;
        networkLiveData(networkStatus) = rounds;
    end

    % tr(transmissions) = operating_nodes;
    opNodes(rounds) = operating_nodes;
    dNodes(rounds) = dead_nodes;
    cNodes(rounds) = CHeads;
    threshData(rounds) = totalData;

    if energy > 0
        nrg(transmissions) = energy;
    end

end

sum = 0;

for i = 1:flagFirstDead
    sum = nrg(i) + sum;
end

% % Plotting Simulation Results "Operating Nodes per Round" %
% figure(2)
% plot(1:rounds, opNodes(1:rounds), '-b', 'Linewidth', 2);
% title ({'LEACH RB NEW'; 'Operating Nodes per Round'; })
% xlabel 'Rounds';
% ylabel 'Operational Nodes';
% hold on;

% figure(3)
% plot(1:rounds, dNodes(1:rounds), '-b', 'Linewidth', 2);
% title ({'LEACH RB NEW'; 'Dead Nodes per Round'; })
% xlabel 'Rounds';
% ylabel 'Dead Nodes';
% hold on;

% figure(4)
% plot(1:rounds, cNodes(1:rounds), '-b', 'Linewidth', 2);
% title ({'LEACH RB NEW'; 'Clusters per Round'; })
% xlabel 'Rounds';
% ylabel 'Clusters';
% hold on;

% figure(5)
% plot(1:rounds, threshData(1:rounds), '-b', 'Linewidth', 2);
% title ({'LEACH RB NEW'; 'Threshold Data per Round'; })
% xlabel 'Rounds';
% ylabel 'Threshold Data';
% hold on;

% figure(6)
% plot(1:flagFirstDead, nrg(1:flagFirstDead), '-b', 'Linewidth', 2');
% title ({'LEACH RB NEW'; 'Energy consumed per Transmission'; })
% xlabel 'Transmission';
% ylabel 'Energy ( J )';
% hold on;

% figure(7)
% bar(1:networkStatus, networkLiveData(1:networkStatus));
% title ({'LEACH RB NEW'; 'Network Status vs Rounds'; })
% xlabel 'Network Status';
% ylabel 'Rounds';
% hold on;

% figure(8)
% plot(1:rounds, remainingEnergy(1:rounds), '-b', 'Linewidth', 2');
% title ({'LEACH RB NEW'; 'Remaining Energy per Round'; })
% xlabel 'Rounds';
% ylabel 'Remaining Energy';
% hold on;

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

% using our algorithm to assign cluster heads
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

    % printing proximity of each CH %
    fid = fopen('proximity.txt', 'a');
    fprintf(fid, '\nRound %d\n', rounds);

    for i = 1:CHeads
        fprintf(fid, 'Proximity of CH(%d) = [', i);

        for j = 1:length(CH_list(i).proximity)
            fprintf(fid, '%d, ', CH_list(i).proximity(j));
        end

        fprintf(fid, ']\n');
    end

    fclose(fid);

end

% function to calculate the number of hops for each CH
function hops = calculate_hops(CH_list, index, indices, hops, rounds)
    fid = fopen('debug.txt', 'a');

    if (CH_list(index).proximity(1) == 0)
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
            fprintf('no path available\n');
        else

            if (rounds == 1)

                fprintf(fid, 'going from index = %d to index = %d\n', index, nextIndex);

                for i = 1:length(indices)
                    fprintf(fid, '%d, ', indices(i));
                end

            end

            indices = [indices, nextIndex];
            hops = calculate_hops(CH_list, nextIndex, indices, hops, rounds) + 1;
        end

    end

    fprintf(fid, '\n');
    fclose(fid);

end

% driver function to calculate the number of hops for each CH
function CH_list = CH_list_with_HOPS(CH_list, CHeads, rad, rounds)

    CH_list = CH_list_with_proximity(CH_list, CHeads, rad, rounds);
    CH_list = arrayfun(@(x) setfield(x, 'hops', 0), CH_list);

    fid = fopen('hops.txt', 'a');
    fprintf(fid, '\nRound %d\n', rounds);
    totalHops = 0;

    % calculating the hops of each CH from the sink
    for i = 1:CHeads

        % if the CH is directly within the communication range of the sink
        if (is_found(CH_list(i).proximity, 0) == 1)
            CH_list(i).hops = 1;

        else
            indices = [];
            indices = [indices, i];
            hops = 0;
            hops = calculate_hops(CH_list, i, indices, hops, rounds);

            CH_list(i).hops = hops;
        end

        fprintf(fid, 'Hops of CH(%d) = %d\n', i, CH_list(i).hops);
        totalHops = totalHops + CH_list(i).hops;

    end

    fprintf(fid, 'ABPL = %f\n', totalHops / (CHeads * (CHeads + 1) / 2));
    fclose(fid);

end
