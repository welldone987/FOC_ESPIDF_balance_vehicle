function pmsm_average_sfunc(block)
%PMSM_AVERAGE_SFUNC Continuous dq average model with abc voltage interface.

setup(block);
end

function setup(block)
block.NumDialogPrms = 7;
block.NumInputPorts = 1;
block.NumOutputPorts = 1;

block.InputPort(1).Dimensions = 4;  % [Va Vb Vc loadTorque]
block.InputPort(1).DirectFeedthrough = false;
block.OutputPort(1).Dimensions = 6; % [Ia Ib Ic omega thetaM thetaE]

block.NumContStates = 4;            % [Id Iq omegaM thetaM]
block.SampleTimes = [0 0];
block.SimStateCompliance = 'DefaultSimState';

block.RegBlockMethod('InitializeConditions', @initializeConditions);
block.RegBlockMethod('Outputs', @outputs);
block.RegBlockMethod('Derivatives', @derivatives);
end

function initializeConditions(block)
block.ContStates.Data = zeros(4, 1);
end

function outputs(block)
x = block.ContStates.Data;
polePairs = block.DialogPrm(1).Data;
id = x(1);
iq = x(2);
omegaM = x(3);
thetaM = x(4);
thetaE = polePairs * thetaM;

cosTheta = cos(thetaE);
sinTheta = sin(thetaE);
iAlpha = cosTheta * id - sinTheta * iq;
iBeta = sinTheta * id + cosTheta * iq;

ia = iAlpha;
ib = -0.5 * iAlpha + (sqrt(3.0) / 2.0) * iBeta;
ic = -0.5 * iAlpha - (sqrt(3.0) / 2.0) * iBeta;
block.OutputPort(1).Data = [ia; ib; ic; omegaM; thetaM; thetaE];
end

function derivatives(block)
u = block.InputPort(1).Data;
x = block.ContStates.Data;

polePairs = block.DialogPrm(1).Data;
Rs = block.DialogPrm(2).Data;
Ld = block.DialogPrm(3).Data;
Lq = block.DialogPrm(4).Data;
flux = block.DialogPrm(5).Data;
inertia = block.DialogPrm(6).Data;
viscousFriction = block.DialogPrm(7).Data;

va = u(1);
vb = u(2);
vc = u(3);
loadTorque = u(4);
id = x(1);
iq = x(2);
omegaM = x(3);
thetaE = polePairs * x(4);
omegaE = polePairs * omegaM;

vAlpha = (2.0 / 3.0) * (va - 0.5 * vb - 0.5 * vc);
vBeta = (2.0 / 3.0) * ((sqrt(3.0) / 2.0) * (vb - vc));
cosTheta = cos(thetaE);
sinTheta = sin(thetaE);
vd = cosTheta * vAlpha + sinTheta * vBeta;
vq = -sinTheta * vAlpha + cosTheta * vBeta;

did = (vd - Rs * id + omegaE * Lq * iq) / Ld;
diq = (vq - Rs * iq - omegaE * (Ld * id + flux)) / Lq;
electromagneticTorque = 1.5 * polePairs * ...
    (flux * iq + (Ld - Lq) * id * iq);
domegaM = (electromagneticTorque - loadTorque - ...
    viscousFriction * omegaM) / inertia;
dthetaM = omegaM;

block.Derivatives.Data = [did; diq; domegaM; dthetaM];
end

