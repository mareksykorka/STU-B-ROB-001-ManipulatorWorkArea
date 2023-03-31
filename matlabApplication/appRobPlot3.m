function appRobPlot3(~, drawLocation, from, where)
    plot3(drawLocation,[from(1),where(1)],[from(2),where(2)],[from(3),where(3)],'LineWidth',5,'Color','yellow')
end