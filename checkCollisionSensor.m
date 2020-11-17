function itDoes = checkCollisionSensor(C1)
    if(checkCollision(C1, 30, [308 264], 60) || checkCollision(C1, 30, [267 539], 60) || checkCollision(C1, 30, [684 279], 60) || checkCollision(C1, 30, [902 100], 60))
       itDoes = 1;
    else
       itDoes = 0;
    end
end

