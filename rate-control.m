%% 802.11 Dynamic Rate Control Simulation
%
% This example shows dynamic rate control by varying the Modulation and
% Coding scheme (MCS) of successive packets transmitted over a frequency
% selective multipath fading channel.

% Copyright 2016-2022 The MathWorks, Inc.
cfgVHT = wlanVHTConfig;         
cfgVHT.ChannelBandwidth = 'CBW40'; % 40 MHz channel bandwidth
cfgVHT.MCS = 1;                    % QPSK rate-1/2
cfgVHT.APEPLength = 4096;          % APEP length in bytes

% Set random stream for repeatability of results
s = rng(21);

tgacChannel = wlanTGacChannel;
tgacChannel.DelayProfile = 'Model-D';
tgacChannel.ChannelBandwidth = cfgVHT.ChannelBandwidth;
tgacChannel.NumTransmitAntennas = 1;
tgacChannel.NumReceiveAntennas = 1;
tgacChannel.TransmitReceiveDistance = 20; % Distance in meters for NLOS
tgacChannel.RandomStream = 'mt19937ar with seed';
tgacChannel.Seed = 0;
tgacChannel.NormalizeChannelOutputs = false;

check_new_algo = false;
if check_new_algo 
    disp("Our algo")
end

% Set the sampling rate for the channel
sr = wlanSampleRate(cfgVHT);
tgacChannel.SampleRate = sr;

rcaAttack = 1;  % Control the sensitivity when MCS is increasing
rcaRelease = 0; % Control the sensitivity when MCS is decreasing
threshold = [11 14 19 20 25 28 30 31 35]; 
snrUp = [threshold inf]+rcaAttack;
snrDown = [-inf threshold]-rcaRelease;
snrInd = cfgVHT.MCS; % Store the start MCS value

% Vars for minstrel

% Editable Params
minstrelStats.ewmaLevel = 0.75;
minstrelStats.updateInterval = 10;
minstrelStats.histReset = 40;

% Defaults
minstrelStats.availableMCS = 0:9; % what we have in 802.11ac

% Taken from here https://community.cisco.com/t5/wireless-mobility-knowledge-base/802-11ac-mcs-rates/ta-p/3155920
% Also measured since we have this 0.5 idle time, can be adjusted for real
% life or other measurements
% Same order as MCS



minstrelStats.attempts = zeros(1, length(minstrelStats.availableMCS));
minstrelStats.success = zeros(1, length(minstrelStats.availableMCS));
minstrelStats.succProb = 0.1 * ones(1, length(minstrelStats.availableMCS));
minstrelStats.ewma = 0.1 * ones(1, length(minstrelStats.availableMCS));
minstrelStats.throughput = zeros(1, length(minstrelStats.availableMCS));

minstrelStats.maxJump = 4;
minstrelStats.chainIndex = 1; 
minstrelStats.retryChain = [0, 0, 0, 0];

minstrelStats.packetCount = 0;


numPackets = 1000; % Number of packets transmitted during the simulation 
walkSNR = true; 

% Select SNR for the simulation
if walkSNR
    meanSNR = 30;   % Mean SNR
    amplitude = 2; % Variation in SNR around the average mean SNR value
    % Generate varying SNR values for each transmitted packet
    baseSNR = sin(linspace(1,10,numPackets))*amplitude+meanSNR;
    snrWalk = baseSNR(1); % Set the initial SNR value
    % The maxJump controls the maximum SNR difference between one
    % packet and the next 
    maxJump = 0.05;
else
    % Fixed mean SNR value for each transmitted packet. All the variability
    % in SNR comes from a time varying radio channel
    snrWalk = 22; %#ok<UNRCH>
end

% To plot the equalized constellation for each spatial stream set
% displayConstellation to true
displayConstellation = false;
if displayConstellation
    ConstellationDiagram = comm.ConstellationDiagram; %#ok<UNRCH>
    ConstellationDiagram.ShowGrid = true;
    ConstellationDiagram.Name = 'Equalized data symbols';
end

% Define simulation variables
snrMeasured = zeros(1,numPackets);
MCS = zeros(1,numPackets);
ber = zeros(1,numPackets);
packetLength = zeros(1,numPackets);


for numPkt = 1:numPackets 
    if walkSNR
        % Generate SNR value per packet using random walk algorithm biased
        % towards the mean SNR
        snrWalk = 0.9*snrWalk+0.1*baseSNR(numPkt)+rand(1)*maxJump*2-maxJump;
    end
    
    % Generate a single packet waveform
    txPSDU = randi([0,1],8*cfgVHT.PSDULength,1,'int8');
    txWave = wlanWaveformGenerator(txPSDU,cfgVHT,'IdleTime',5e-4);
    
    % Receive processing, including SNR estimation
    y = processPacket(txWave,snrWalk,tgacChannel,cfgVHT);
    
    % Plot equalized symbols of data carrying subcarriers
    if displayConstellation && ~isempty(y.EstimatedSNR)
        release(ConstellationDiagram);
        ConstellationDiagram.ReferenceConstellation = wlanReferenceSymbols(cfgVHT);
        ConstellationDiagram.Title = ['Packet ' int2str(numPkt)];
        ConstellationDiagram(y.EqDataSym(:));
        drawnow 
    end
    
    % Store estimated SNR value for each packet
    if isempty(y.EstimatedSNR) 
        snrMeasured(1,numPkt) = NaN;
    else
        snrMeasured(1,numPkt) = y.EstimatedSNR;
    end
    
    % Determine the length of the packet in seconds including idle time
    packetLength(numPkt) = y.RxWaveformLength/sr;
    success = true;
    % Calculate packet error rate (PER)
    if isempty(y.RxPSDU)
        % Set the PER of an undetected packet to NaN
        ber(numPkt) = NaN;
    else
        [numErrors ,ber(numPkt)] = biterr(y.RxPSDU, txPSDU);
        if numErrors > 0
            success = false;
        end
    end

    % Compare the estimated SNR to the threshold, and adjust the MCS value
    % used for the next packet
    MCS(numPkt) = cfgVHT.MCS; % Store current MCS value
    increaseMCS = (mean(y.EstimatedSNR) > snrUp((snrInd==0)+snrInd));
    decreaseMCS = (mean(y.EstimatedSNR) <= snrDown((snrInd==0)+snrInd));
    snrInd = snrInd+increaseMCS-decreaseMCS;
    
    % cfgVHT.MCS = 0;
    if check_new_algo
        [newMCS, newStats] = minstrelRateControl(success, minstrelStats, cfgVHT.MCS, mean(y.EstimatedSNR));
        minstrelStats = newStats;
    else 
        newMCS = snrInd-1;
    end
    cfgVHT.MCS = newMCS;
    
end


% Display and plot simulation results
disp(['Overall data rate: ' num2str(8*cfgVHT.APEPLength*(numPackets-numel(find(ber)))/sum(packetLength)/1e6) ' Mbps']);
disp(['Overall packet error rate: ' num2str(numel(find(ber))/numPackets)]);

plotResults(ber,packetLength,snrMeasured,MCS,cfgVHT);

% Restore default stream
rng(s);




function Y = processPacket(txWave,snrWalk,tgacChannel,cfgVHT)
    % Pass the transmitted waveform through the channel, perform
    % receiver processing, and SNR estimation.
    
    chanBW = cfgVHT.ChannelBandwidth; % Channel bandwidth
    % Set the following parameters to empty for an undetected packet
    estimatedSNR = [];
    eqDataSym = [];
    noiseVarVHT = [];
    rxPSDU = [];
    
    % Get the OFDM info
    ofdmInfo = wlanVHTOFDMInfo('VHT-Data',cfgVHT);
    
    % Pass the waveform through the fading channel model
    rxWave = tgacChannel(txWave);
    
    % Account for noise energy in nulls so the SNR is defined per
    % active subcarrier
    packetSNR = snrWalk-10*log10(ofdmInfo.FFTLength/ofdmInfo.NumTones);
    
    % Add noise
    rxWave = awgn(rxWave,packetSNR);
    rxWaveformLength = size(rxWave,1); % Length of the received waveform
    
    % Recover packet
    ind = wlanFieldIndices(cfgVHT); % Get field indices
    pktOffset = wlanPacketDetect(rxWave,chanBW); % Detect packet
    
    if ~isempty(pktOffset) % If packet detected
        % Extract the L-LTF field for fine timing synchronization
        LLTFSearchBuffer = rxWave(pktOffset+(ind.LSTF(1):ind.LSIG(2)),:);
    
        % Start index of L-LTF field
        finePktOffset = wlanSymbolTimingEstimate(LLTFSearchBuffer,chanBW);
     
        % Determine final packet offset
        pktOffset = pktOffset+finePktOffset;
        
        if pktOffset<15 % If synchronization successful
            % Extract VHT-LTF samples from the waveform, demodulate and
            % perform channel estimation
            VHTLTF = rxWave(pktOffset+(ind.VHTLTF(1):ind.VHTLTF(2)),:);
            demodVHTLTF = wlanVHTLTFDemodulate(VHTLTF,cfgVHT);
            [chanEstVHTLTF,chanEstSSPilots] = wlanVHTLTFChannelEstimate(demodVHTLTF,cfgVHT);
            
            % Extract VHT data field
            vhtdata = rxWave(pktOffset+(ind.VHTData(1):ind.VHTData(2)),:);
            
            % Estimate the noise power in VHT data field
            noiseVarVHT = vhtNoiseEstimate(vhtdata,chanEstSSPilots,cfgVHT);
            
            % Recover equalized symbols at data carrying subcarriers using
            % channel estimates from VHT-LTF
            [rxPSDU,~,eqDataSym] = wlanVHTDataRecover(vhtdata,chanEstVHTLTF,noiseVarVHT,cfgVHT);
            
            % SNR estimation per receive antenna
            powVHTLTF = mean(VHTLTF.*conj(VHTLTF));
            estSigPower = powVHTLTF-noiseVarVHT;
            estimatedSNR = 10*log10(mean(estSigPower./noiseVarVHT));
        end
    end
    
    % Set output
    Y = struct( ...
        'RxPSDU',           rxPSDU, ...
        'EqDataSym',        eqDataSym, ...
        'RxWaveformLength', rxWaveformLength, ...
        'NoiseVar',         noiseVarVHT, ...
        'EstimatedSNR',     estimatedSNR);
    
end

function plotResults(ber,packetLength,snrMeasured,MCS,cfgVHT)
    % Visualize simulation results

    figure('Outerposition',[50 50 900 700])
    subplot(4,1,1);
    plot(MCS);
    xlabel('Packet Number')
    ylabel('MCS')
    title('MCS selected for transmission')

    subplot(4,1,2);
    plot(snrMeasured);
    xlabel('Packet Number')
    ylabel('SNR')
    title('Estimated SNR')

    subplot(4,1,3);
    plot(find(ber==0),ber(ber==0),'x') 
    hold on; stem(find(ber>0),ber(ber>0),'or') 
    if any(ber)
        legend('Successful decode','Unsuccessful decode') 
    else
        legend('Successful decode') 
    end
    xlabel('Packet Number')
    ylabel('BER')
    title('Instantaneous bit error rate per packet')

    subplot(4,1,4);
    windowLength = 3; % Length of the averaging window
    movDataRate = movsum(8*cfgVHT.APEPLength.*(ber==0),windowLength)./movsum(packetLength,windowLength)/1e6;
    plot(movDataRate)
    xlabel('Packet Number')
    ylabel('Mbps')
    title(sprintf('Throughput over last %d packets',windowLength))
    
end



% Below is our Minstrel implementation

% Throughput = Rate (x Mpbs) * Succes probability

% EWMA = old * factor + new * (1 - factor)

% Retry chain
% random < best | random > best
% Best througput| Random
% Random        | Best throughput
% Best probs    | Best probs
% Lowest        | Lowest

% How it works?
% 0. Create tables with EWMA probabilities for all rates
% 1. Pick lookaround rate from available rates
%       * If better than current, try it
%       * If worse, place 2nd
% 2. See what happens
%       * Success, record it in table
%       * Fail, record, continue in retry chain
% 3. If x packets send, recalculate retry chain
% 4. Repeat until all packets send



function [newMCS, minstrelStats] = minstrelRateControl(success, minstrelStats, prevMCS, estimatedSNR)

% Perform the periodic update of EWMA table. if neccessary

if  mod(minstrelStats.packetCount, minstrelStats.updateInterval) == 0
    % disp("---------------------------Chain calc");
    bestProbability = -1;
    bestProbabilityIndex = 1;
    % Go through every MCS we have
    for i = 1:length(minstrelStats.availableMCS)
        if minstrelStats.attempts(i) > 0 % div below
            currentProbability = minstrelStats.success(i) / minstrelStats.attempts(i);
            
            minstrelStats.ewma(i) = (minstrelStats.ewmaLevel * minstrelStats.ewma(i) ...
                + (1 - minstrelStats.ewmaLevel) * currentProbability);
            
            % Check best probability
            if minstrelStats.ewma(i) >= bestProbability
                bestProbability = minstrelStats.ewma(i);
                bestProbabilityIndex = i;
            end

            % Reset at the end
            if mod(minstrelStats.packetCount, minstrelStats.histReset) == 0
                minstrelStats.attempts(i) = 0;
                minstrelStats.success(i) = 0;
                minstrelStats.packetCount = 0;
            end
        end
    end

    % Recalculate throughput for each
    minstrelStats.bitrates = [11, 18, 24, 28, 34, 38, 39, 41, 43, 44]; % For some reason this needs to be here otherwise matlab dies
    minstrelStats.throughput = minstrelStats.bitrates .* minstrelStats.ewma;

    % disp("throughputs");
    % disp(minstrelStats.throughput);

    % Now we want to construct the chain
    % First find best throughput
    [~, sortedIndices] = sort(minstrelStats.throughput, 'descend');

    bestThroughput = minstrelStats.bitrates(sortedIndices(1));
    bestThroughputIndex = sortedIndices(1) - 1;
    % disp("Best throughput is " + bestThroughputIndex + " and it's rate " + bestThroughput);

    % Dora the explorer packet
    % Pick random, if better then throughput insert it higher
    random_mcs = randi([1, min(bestThroughputIndex + minstrelStats.maxJump, 10)]);
    random_rate = minstrelStats.bitrates(random_mcs);
    % disp("Random MCS is " + random_mcs + " and it's rate " + random_rate);

    if random_rate > bestThroughput && minstrelStats.chainIndex < 2 % If we crashed, let's not immediately try again
        % disp("random won");
        % random, Best throughput, best probs, then safest
        minstrelStats.retryChain = [(random_mcs - 1), bestThroughputIndex, (bestProbabilityIndex - 1), max(bestThroughputIndex - minstrelStats.maxJump, 0)];
    else
        % disp("random lost");
        % Best throughput, random, best probs, then safest
        minstrelStats.retryChain = [bestThroughputIndex, (random_mcs - 1), (bestProbabilityIndex - 1), max(bestThroughputIndex - minstrelStats.maxJump, 0)];
    end

    % Prepare to send
    % Reset retry chain position
    % disp(minstrelStats.retryChain);
    minstrelStats.chainIndex = 1;
    
end
% Also remember to update stats
minstrelStats.attempts(prevMCS + 1) = minstrelStats.attempts(prevMCS + 1) + 1;
if success 
    minstrelStats.success(prevMCS + 1) = minstrelStats.attempts(prevMCS + 1) + 1;
end
% Continue along chain

% If success, stay in place, return the mcs in our chain id
% If fail, move lower if possible


if ~success && minstrelStats.chainIndex < 4
    % disp("prev failed - going down chain");
    minstrelStats.chainIndex = minstrelStats.chainIndex + 1;
end

% First take count of what packet we're on rn
% disp("chain index is " + minstrelStats.chainIndex);
minstrelStats.packetCount = minstrelStats.packetCount + 1; % I'd like to heavily complain matlab has no += ;'(
newMCS = minstrelStats.retryChain(minstrelStats.chainIndex);
% disp("Picked " + newMCS);

end




