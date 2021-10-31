function [pk, xk] = solution(Zmp, N)
%SOLUTION Summary of this function goes here
%   Detailed explanation goes here
global A B C T Q R K P;

% xk = [x x' x'']; (eq. 4.72)
xk = zeros(3,length(T));
uk = 0; % uk initialization
lim = length(T); % time limite (size)

% Implementation of fi (Eq. 4.75)
f = zeros(1,N);
for i = 1:N
    % dlqr assumed C = 1, Then Q = 1 here.
    f(i) = (R+B'*P*B)\B'*(transpose(A-B*K)^(i-1))*C'*Q;
end

% Implementation of the system
pref = zeros(N,1); % pref for eq. 4.74
pk = zeros(1,lim); %pk for eq. 4.72
for k = 1:lim
    % Creating p_ref for eq. 4.74
    for i = 1:N
        if (k+i <= lim)
            pref(i,1) = Zmp(k+i);
        else
            pref(i,1) = Zmp(lim);
        end
    end
    % implementation of eq 4.72
    if(k ~= lim)
        xk(:,k+1) = A*xk(:,k) + B*uk;
    end
    uk = -K*xk(:,k) + f*pref;
    pk(k) = C*xk(:,k); 
end 

end

