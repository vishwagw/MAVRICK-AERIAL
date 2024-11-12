function LS = calls(path)
% This function calculates the maximum flight direction change during the entire process
[n,~] = size(path);
mu1 = 0.5; mu2 = 0.5; % Weighting factors to yaw and climb angles
Max = 0;

for i = 2 : n-1
    qBefore = path(i-1,:);
    q = path(i,:);
    qNext = path(i+1,:);
    % Calculate qBefore to q track anglex1, gam1
    qBefore2q = q - qBefore;
    gam1 = asin(qBefore2q(3)/sqrt(sum(qBefore2q.^2)));
    if qBefore2q(1) ~= 0 || qBefore2q(2) ~= 0
        x1 = asin(abs(qBefore2q(2)/sqrt(qBefore2q(1)^2+qBefore2q(2)^2)));
    else
        x1 = 0;
    end
    % Calculate the track angle from q to qNextx2, gam2
    q2qNext = qNext - q;
    gam2 = asin(q2qNext(3)/sqrt(sum(q2qNext.^2)));
    if q2qNext(1) ~= 0 || q2qNext(2) ~= 0
        x2 = asin(abs(q2qNext(2)/sqrt(q2qNext(1)^2+q2qNext(2)^2)));
    else
        x2 = 0;
    end
    if qBefore2q(1) > 0 && qBefore2q(2) > 0
        x1 = x1;
    end
    if qBefore2q(1) < 0 && qBefore2q(2) > 0
        x1 = pi - x1;
    end
    if qBefore2q(1) < 0 && qBefore2q(2) < 0
        x1 = pi + x1;
    end
    if qBefore2q(1) > 0 && qBefore2q(2) < 0
        x1 = 2*pi - x1;
    end
    if qBefore2q(1) > 0 && qBefore2q(2) == 0
        x1 = 0;
    end
    if qBefore2q(1) == 0 && qBefore2q(2) > 0
        x1 = pi/2;
    end
    if qBefore2q(1) < 0 && qBefore2q(2) == 0
        x1 = pi;
    end
    if qBefore2q(1) == 0 && qBefore2q(2) < 0
        x1 = 3*pi/2;
    end

    if q2qNext(1) > 0 && q2qNext(2) > 0
        x2 = x2;
    end
    if q2qNext(1) < 0 && q2qNext(2) > 0
        x2 = pi - x2;
    end
    if q2qNext(1) < 0 && q2qNext(2) < 0
        x2 = pi + x2;
    end
    if q2qNext(1) > 0 && q2qNext(2) < 0
        x2 = 2*pi - x2;
    end
    if q2qNext(1) > 0 && q2qNext(2) == 0
        x2 = 0;
    end
    if q2qNext(1) == 0 && q2qNext(2) > 0
        x2 = pi/2;
    end
    if q2qNext(1) < 0 && q2qNext(2) == 0
        x2 = pi;
    end
    if q2qNext(1) == 0 && q2qNext(2) < 0
        x2 = 3*pi/2;
    end

    % Determine whether the angle change exceeds
    if abs(x1-x2)>=pi
        deltaX = 2*pi-abs(x1-x2);
    else
        deltaX = abs(x1-x2);
    end
    if mu1 * abs(deltaX) + mu2 * abs(gam2 - gam1) > Max
        Max = mu1 * abs(deltaX) + mu2 * abs(gam2 - gam1);
    end
end
LS = Max;
end



