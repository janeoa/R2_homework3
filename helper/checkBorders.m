function pos = checkBorders(newPos)
            x = newPos(1);
            y = newPos(2);
            offsetX = 101;
            offsetY = 101;
            
            if(x>1000-offsetX)
                x = 1000-offsetX;
            end
            if(x<1+offsetX)
                x = 1+offsetX;
            end
            
            if(y>663-offsetY)
                y = 663-offsetY;
            end
            if(y<1+offsetY)
                y = 1+offsetY;
            end
            pos = [x y];
        end