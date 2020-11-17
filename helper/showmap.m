function showmap(show_background, grid)
    
    hold on;
    if(show_background)
        imshow(imread("map.png"));
    end
    
    toShow = 1-flipud(transpose(grid));
    h = imshow(toShow);
    set(h, 'AlphaData', 1-toShow-0.3);
    hold off;
end

