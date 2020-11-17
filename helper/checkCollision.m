function itDoes = checkCollision(C1, R1, C2, R2)
    distance = sqrt((C1(1)-C2(1))^2+(C1(2)-C2(2))^2);
    itDoes = R1+R2 > distance;
end

