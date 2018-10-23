function find_calibration_parameters()
    %----------------------------------------------------------------------
    % Constants
    %----------------------------------------------------------------------
    % readings at 0 qc.
    % Since it has different values, it means when the EPOS was turned on,
    % the visually estimated position for wheels with zero angle has a
    % small offset. In this case should be near atan2d(0.5*0.05, 1.47) ~ 1 
    % degree
    steeringWheelZero=0.01*[92.5, 97.5];
    % dL is distance from 0R and 0L with steering wheel at center position.
    % qc should be near zero.
    dL = 0.01*45.5;
    % d1 is distance between attached laser position in the outside wheel 
    % face
    d1 = 1.445;
    % d0 is distance between center or wheels. Shaft distance.
    d0 = 1.40;
    % h1 is distance from wheel front axis to the "rule" of readings.
    h1 = 1.47;
    % maxR and maxL is maximum distance achievable from poit 0R and 0L
    % respectivly
    maxR = 2.19;
    maxL = 2.19;
    maxRange = maxR+maxL-dL;
    
    L1 = 0.5*(maxRange -d1);
    L2 = 0.5*(d1-dL);
    %----------------------------------------------------------------------
    % kept for future improvements!
    %----------------------------------------------------------------------
    %    %marked positions of left wheel starting from 0_L to the left side
    %    distL =0.01*(0:20:200);
    %    % readings of encoder
    %    qcL = [47602, 40267, 30764, 20356, 9374, -1643, -11968, -21040,...
    %           -29419, -36495, -41506];
    %----------------------------------------------------------------------
    % readings right wheel starting from 0_R to the right side
    distR =0.01*(0:20:200);
    qcR = [ -45178, -37680, -28428, -18063, -6735, 4388, 15110, 24331, ...
        32335, 39414, 44218];
    % readings of left wheel from 0L to the left, placing the steering 
    % wheel in the same quadrature counters as best as possible  
    distL=[219, 184.5, 158, 133.5, 111, 91, 71.8, 53.8, 37.8, 23, 10];
    
    %----------------------------------------------------------------------
    % Data correction to fit 0 with steering wheel aligned
    %----------------------------------------------------------------------
    % adjust measures so \beta = 0 gives a reading of 0 distance.
    %distL = distL-(dL+L2);
    distL = 0.01*distL-(dL+L2);
    % make positive to left side and 
    % adjust measures so \beta = 0 gives a reading of 0 distance.
    distR = -1*(distR-(dL+L2));
    %----------------------------------------------------------------------
    % calculate all angles
    %----------------------------------------------------------------------
    betaL = zeros(1,length(distL));
    alphaR = betaL;
    dirac = alphaR;
    deltaDirac = dirac;
    for I =1: length(distL)
        % betaL(I) = atan2d(distL(I), 1.47);
        betaL(I) = atan2d(distL(I),1.47);
        alphaR(I) = atan2d(distR(I), 1.47);
        dirac(I) = 0.5*(betaL(I)+alphaR(I));
        deltaDirac(I) = 0.5*(betaL(I)-alphaR(I));
    end
    %----------------------------------------------------------------------
    % plot figures
    %----------------------------------------------------------------------
    figure();
    plot(qcR,distL,qcR,distR);
    legend('Dist_L','Dist_R');
    ylabel('distance [m]');
    xlabel('Steering wheel position [qc]');
    
    figure();
    plot(qcR, alphaR, qcR, betaL, qcR, dirac, qcR, deltaDirac);
    xlabel('Steering wheel position [qc]');
    ylabel('angles [degrees]');
    hold on;
    line([qcR(1) qcR(end)],[0, 0],'color', 'red','LineStyle','--');
    limMax = max(max(alphaR),max(betaL));
    limMin = min(min(alphaR),min(betaL));
    line([qcR(1)+0.5*(qcR(end)-qcR(1)) qcR(1)+0.5*(qcR(end)-qcR(1))],...
        [limMin limMax],'color','red','lineStyle','--');
    legend('\alpha_R','\beta_L', '\delta','\Delta\delta','0');
    
    % change format to show small values in cmd window
    format short e;
    %----------------------------------------------------------------------
    % polyfit of alpha, beta, dirac, deltaDirac 
    %----------------------------------------------------------------------
    % alpha and beta must have oposite concavities
    
    [pAlpha, sAlpha] = polyfit(qcR, alphaR, 2);
    fprintf('Polyfit for alpha:');
    display(pAlpha);
    fprintf('Norm of the residuals:%.4g\n', sAlpha.normr)
    [pBeta, sBeta]  = polyfit(qcR, betaL, 2);
    fprintf('Polyfit for beta:');
    display(pBeta);
    fprintf('Norm of the residuals:%.4g\n', sBeta.normr)
    % delta must be concave up always.
    [pDirac, sDirac] = polyfit(qcR, dirac, 1);
    fprintf('Polyfit for dirac:');
    display(pDirac);
    fprintf('Norm of the residuals:%.4g\n', sDirac.normr)
    [pDelta, sDelta] = polyfit(qcR, deltaDirac, 2);
    fprintf('Polyfit for delta:');
    display(pDelta);
    fprintf('Norm of the residuals:%.4g\n', sDelta.normr)
    % reset format
    format;
    %----------------------------------------------------------------------
    % evaluate polyfits for range -40000 to 40000
    %----------------------------------------------------------------------
    qc = -40000:100:40000;
    % set zero offset in each polyfit
    % The values for alpha = beta = dirac = deltaDirac must be all zero por
    % a perfect zero alignemt of the steering wheels. This does not happen
    % because the wheels at qc = 0 had a small misalignment.
    pAlpha(end) = 0;
    pBeta(end) = 0;
    pDirac(end) = 0;
    pDelta(end) = 0;
    alpha = polyval(pAlpha, qc);
    beta = polyval(pBeta, qc);
    dirac2 = polyval(pDirac, qc);
    delta = polyval(pDelta, qc);
    figure();
    subplot(2,1,1);
    plot(qc, alpha, qc, beta, qc, dirac2);
    % xlabel('Quadrature counters [qc]');
    ylabel('Angle [degrees]');
    
    title(['Steering wheel ratio estimate: \delta = ',...
        sprintf('%.4g', pDirac(1)),'qc']);
    legend('\alpha','\beta', '\delta');
    subplot(2,1,2);
    plot(qc, delta);
    xlabel('Quadrature counters [qc]');
    ylabel('Angle [degrees]');
    legend('\Delta\delta');
end