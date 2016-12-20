% This function computes the muscle fiber length from the normalized tendon
% force

function [lM,lMtilde ] = FiberLength_Ftilde(Ftilde,params,lMT)

lMo = ones(size(Ftilde,1),1)*params(2,:);
lTs = ones(size(Ftilde,1),1)*params(3,:);
alphao = ones(size(Ftilde,1),1)*params(4,:);

% Non-linear tendon
lTtilde = real(log(5*(Ftilde + 0.25))/35 + 0.995);

% Hill-model relationship
lM = sqrt((lMo.*sin(alphao)).^2+(lMT-lTs.*lTtilde).^2);
lMtilde = lM./lMo;
end

