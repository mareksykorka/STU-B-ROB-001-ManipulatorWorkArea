% Encapsulates the plot3 function of native matlab for better code readability to CS of robotic manipulator.
function appRobPlotCS(~,drawLocation, bod, cs, from, where)
    plot3(drawLocation,[bod(1,from),cs.i(1,where)],[bod(2,from),cs.i(2,where)],[bod(3,from),cs.i(3,where)],'Color','red','LineWidth',2)
    plot3(drawLocation,[bod(1,from),cs.j(1,where)],[bod(2,from),cs.j(2,where)],[bod(3,from),cs.j(3,where)],'Color','green','LineWidth',2)
    plot3(drawLocation,[bod(1,from),cs.k(1,where)],[bod(2,from),cs.k(2,where)],[bod(3,from),cs.k(3,where)],'Color','blue','LineWidth',2)
end
