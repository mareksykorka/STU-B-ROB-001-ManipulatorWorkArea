function appMain(app)
    % cs for CoordinateSystem
    cs.i(:,1) = [50; 0; 0; 1];
    cs.j(:,1) = [0; 50; 0; 1];
    cs.k(:,1) = [0; 0; 50; 1];
    bod(:,1) = [0; 0; 0; 1];
    
    cs.i(:,2) = (appRobRotate(app,'z','deg',app.phi1)*cs.i(:,1));
    cs.j(:,2) = (appRobRotate(app,'z','deg',app.phi1)*cs.j(:,1));
    cs.k(:,2) = (appRobRotate(app,'z','deg',app.phi1)*cs.k(:,1));
    
    bod(:,2) = (appRobRotate(app,'z','deg',app.phi1)*appRobTranslate(app,'z',app.l1)*bod(:,1));
    cs.i(:,3) = (appRobRotate(app,'z','deg',app.phi1)*appRobTranslate(app,'z',app.l1)*cs.i(:,1));
    cs.j(:,3) = (appRobRotate(app,'z','deg',app.phi1)*appRobTranslate(app,'z',app.l1)*cs.j(:,1));
    cs.k(:,3) = (appRobRotate(app,'z','deg',app.phi1)*appRobTranslate(app,'z',app.l1)*cs.k(:,1));
    
    cs.i(:,4) = (appRobRotate(app,'z','deg',app.phi1)*appRobTranslate(app,'z',app.l1)*appRobRotate(app,'y','deg',app.phi2)*cs.i(:,1));
    cs.j(:,4) = (appRobRotate(app,'z','deg',app.phi1)*appRobTranslate(app,'z',app.l1)*appRobRotate(app,'y','deg',app.phi2)*cs.j(:,1));
    cs.k(:,4) = (appRobRotate(app,'z','deg',app.phi1)*appRobTranslate(app,'z',app.l1)*appRobRotate(app,'y','deg',app.phi2)*cs.k(:,1));
    
    bod(:,3) = (appRobRotate(app,'z','deg',app.phi1)*appRobTranslate(app,'z',app.l1)*appRobRotate(app,'y','deg',app.phi2)*appRobTranslate(app,'z',app.l2)*bod(:,1));
    cs.i(:,5) = (appRobRotate(app,'z','deg',app.phi1)*appRobTranslate(app,'z',app.l1)*appRobRotate(app,'y','deg',app.phi2)*appRobTranslate(app,'z',app.l2)*cs.i(:,1));
    cs.j(:,5) = (appRobRotate(app,'z','deg',app.phi1)*appRobTranslate(app,'z',app.l1)*appRobRotate(app,'y','deg',app.phi2)*appRobTranslate(app,'z',app.l2)*cs.j(:,1));
    cs.k(:,5) = (appRobRotate(app,'z','deg',app.phi1)*appRobTranslate(app,'z',app.l1)*appRobRotate(app,'y','deg',app.phi2)*appRobTranslate(app,'z',app.l2)*cs.k(:,1));
    
    cs.i(:,6) = (appRobRotate(app,'z','deg',app.phi1)*appRobTranslate(app,'z',app.l1)*appRobRotate(app,'y','deg',app.phi2)*appRobTranslate(app,'z',app.l2)*appRobRotate(app,'y','deg',app.phi3)*cs.i(:,1));
    cs.j(:,6) = (appRobRotate(app,'z','deg',app.phi1)*appRobTranslate(app,'z',app.l1)*appRobRotate(app,'y','deg',app.phi2)*appRobTranslate(app,'z',app.l2)*appRobRotate(app,'y','deg',app.phi3)*cs.j(:,1));
    cs.k(:,6) = (appRobRotate(app,'z','deg',app.phi1)*appRobTranslate(app,'z',app.l1)*appRobRotate(app,'y','deg',app.phi2)*appRobTranslate(app,'z',app.l2)*appRobRotate(app,'y','deg',app.phi3)*cs.k(:,1));
    
    bod(:,4) = (appRobRotate(app,'z','deg',app.phi1)*appRobTranslate(app,'z',app.l1)*appRobRotate(app,'y','deg',app.phi2)*appRobTranslate(app,'z',app.l2)*appRobRotate(app,'y','deg',app.phi3)*appRobTranslate(app,'z',app.l3)*bod(:,1));
    cs.i(:,7) = (appRobRotate(app,'z','deg',app.phi1)*appRobTranslate(app,'z',app.l1)*appRobRotate(app,'y','deg',app.phi2)*appRobTranslate(app,'z',app.l2)*appRobRotate(app,'y','deg',app.phi3)*appRobTranslate(app,'z',app.l3)*cs.i(:,1));
    cs.j(:,7) = (appRobRotate(app,'z','deg',app.phi1)*appRobTranslate(app,'z',app.l1)*appRobRotate(app,'y','deg',app.phi2)*appRobTranslate(app,'z',app.l2)*appRobRotate(app,'y','deg',app.phi3)*appRobTranslate(app,'z',app.l3)*cs.j(:,1));
    cs.k(:,7) = (appRobRotate(app,'z','deg',app.phi1)*appRobTranslate(app,'z',app.l1)*appRobRotate(app,'y','deg',app.phi2)*appRobTranslate(app,'z',app.l2)*appRobRotate(app,'y','deg',app.phi3)*appRobTranslate(app,'z',app.l3)*cs.k(:,1));
    
    xy_iter = 1;
    krok = 5;
    
    xy = zeros(5,360/krok);
    
    for (i = 0:5:360)
        xy(5,xy_iter) = i;
        xy_iter = xy_iter + 1;
    end
    
    for(phi1_local = app.phi1_min:krok:app.phi1_max)
        for(phi2_local = app.phi2_min:krok:app.phi2_max)
            for(phi3_local = app.phi3_min:krok:app.phi3_max)
                Ct(1:4) = (appRobRotate(app,'z','deg',phi1_local)*appRobTranslate(app,'z',app.l1)*appRobRotate(app,'y','deg',phi2_local)*appRobTranslate(app,'z',app.l2)*appRobRotate(app,'y','deg',phi3_local)*appRobTranslate(app,'z',app.l3)*bod(:,1));
                Ct(5) = sqrt(Ct(1)*Ct(1)+Ct(2)*Ct(2));
                Ct(6) = atan2(Ct(2),Ct(1));
                Ct(6) = round(rad2deg(Ct(6)),0);
                if Ct(6) < 0
                    Ct(6) = Ct(6) + 360;
                end
    
                I = find(xy(5,:) == Ct(6));
                if(xy(4,I) < Ct(5))
                    xy(1:2,I) = Ct(1:2);
                    xy(3,I) = app.l1;
                    xy(4,I) = Ct(5);
                end
            end
        end
    end
    xy(1:4,end)=xy(1:4,1);


%     xy_iter = 1;
%     krok = 5;
%     
%     for(phi1_local = app.phi1_min:krok:app.phi1_max)
%         xy(1:4,xy_iter) = 0;
%         for(phi2_local = app.phi2_min:krok:0)
%             for(phi3_local = app.phi3_min:krok:app.phi3_max)
%                 Ct(1:4) = (appRobRotate(app,'z','deg',phi1_local)*appRobTranslate(app,'z',app.l1)*appRobRotate(app,'y','deg',phi2_local)*appRobTranslate(app,'z',app.l2)*appRobRotate(app,'y','deg',phi3_local)*appRobTranslate(app,'z',app.l3)*bod(:,1));
%                 Ct(5) = sqrt(Ct(1)*Ct(1)+Ct(2)*Ct(2));
%                 Ct(6) = asin(Ct(1)/Ct(5));
%                 if(xy(4,xy_iter) < Ct(5) && (Ct(6) < 90 && Ct(6) > -90))
%                     xy(1:2,xy_iter) = Ct(1:2);
%                     xy(3,xy_iter) = app.l1;
%                     xy(4,xy_iter) = Ct(5);
%                 end
%             end
%         end
%         xy_iter = xy_iter + 1;
%     end
%     
%     for(phi1_local = app.phi1_min:krok:app.phi1_max)
%         xy(1:4,xy_iter) = 0;
%         for(phi2_local = 0:krok:app.phi2_max)
%             for(phi3_local = app.phi3_min:krok:app.phi3_max)
%                 Ct(1:4) = (appRobRotate(app,'z','deg',phi1_local)*appRobTranslate(app,'z',app.l1)*appRobRotate(app,'y','deg',phi2_local)*appRobTranslate(app,'z',app.l2)*appRobRotate(app,'y','deg',phi3_local)*appRobTranslate(app,'z',app.l3)*bod(:,1));
%                 Ct(5) = sqrt(Ct(1)*Ct(1)+Ct(2)*Ct(2));
%                 if(xy(4,xy_iter) < Ct(5) && (Ct(6) > 90 && Ct(6) < -90))
%                     xy(1:2,xy_iter) = Ct(1:2);
%                     xy(3,xy_iter) = app.l1;
%                     xy(4,xy_iter) = Ct(5);
%                 end
%             end
%         end
%         xy_iter = xy_iter + 1;
%     end
%     xy(1:4,xy_iter)=xy(1:4,1);
    
    xz_iter = 1;
    krok = 5;
    
    phi1_local = 0;
    phi2_local = app.phi2_min;
    
    for(phi3_local = app.phi3_min:krok:app.phi3_max)
        Ct(1:4) = (appRobRotate(app,'z','deg',phi1_local)*appRobTranslate(app,'z',app.l1)*appRobRotate(app,'y','deg',phi2_local)*appRobTranslate(app,'z',app.l2)*appRobRotate(app,'y','deg',phi3_local)*appRobTranslate(app,'z',app.l3)*bod(:,1));
        Ct(5) = sqrt(Ct(1)*Ct(1)+Ct(2)*Ct(2));
    
        xz(1:3,xz_iter) = Ct(1:3);
        xz(4,xz_iter) = Ct(5);
    
        xz_iter = xz_iter + 1;
    end
    
    phi3_local = app.phi3_max;
    
    for(phi2_local = app.phi2_min:krok:app.phi2_max)
        Ct(1:4) = (appRobRotate(app,'z','deg',phi1_local)*appRobTranslate(app,'z',app.l1)*appRobRotate(app,'y','deg',phi2_local)*appRobTranslate(app,'z',app.l2)*appRobRotate(app,'y','deg',phi3_local)*appRobTranslate(app,'z',app.l3)*bod(:,1));
        Ct(5) = sqrt(Ct(1)*Ct(1)+Ct(2)*Ct(2));
    
        xz(1:3,xz_iter) = Ct(1:3);
        xz(4,xz_iter) = Ct(5);
    
        xz_iter = xz_iter + 1;
    end
    
    phi2_local = app.phi2_max;
    
    for(phi3_local = app.phi3_max:-krok:app.phi3_min)
        Ct(1:4) = (appRobRotate(app,'z','deg',phi1_local)*appRobTranslate(app,'z',app.l1)*appRobRotate(app,'y','deg',phi2_local)*appRobTranslate(app,'z',app.l2)*appRobRotate(app,'y','deg',phi3_local)*appRobTranslate(app,'z',app.l3)*bod(:,1));
        Ct(5) = sqrt(Ct(1)*Ct(1)+Ct(2)*Ct(2));
    
        xz(1:3,xz_iter) = Ct(1:3);
        xz(4,xz_iter) = Ct(5);
    
        xz_iter = xz_iter + 1;
    end
    
    phi3_local = app.phi3_min;
    
    for(phi2_local = app.phi2_max:-krok:app.phi2_min)
        Ct(1:4) = (appRobRotate(app,'z','deg',phi1_local)*appRobTranslate(app,'z',app.l1)*appRobRotate(app,'y','deg',phi2_local)*appRobTranslate(app,'z',app.l2)*appRobRotate(app,'y','deg',phi3_local)*appRobTranslate(app,'z',app.l3)*bod(:,1));
        Ct(5) = sqrt(Ct(1)*Ct(1)+Ct(2)*Ct(2));
    
        xz(1:3,xz_iter) = Ct(1:3);
        xz(4,xz_iter) = Ct(5);
    
        xz_iter = xz_iter + 1;
    end
    
    
    cla(app.UIAxes);
    if((app.drawRM == true)||(app.drawRM == true && app.drawXY == true && app.drawXZ == true))
        view(app.UIAxes,[40 30]);
    elseif (app.drawXY == true)
        view(app.UIAxes,[0 90]);
    elseif (app.drawXZ == true)
        view(app.UIAxes,[0 0]);
    end

    view(app.UIAxes,[0 90]);
    
    if(app.drawRM == true)
        for (iterator = 2:1:length(bod))
            appRobPlot3(app,app.UIAxes,bod(:,iterator-1),bod(:,iterator))
            appRobSphere(app,app.UIAxes,7,bod(:,iterator))
        end
        
        plot3(app.UIAxes,[bod(1,1),cs.i(1,2)],[bod(2,1),cs.i(2,2)],[bod(3,1),cs.i(3,2)],'Color','red','LineWidth',2)
        plot3(app.UIAxes,[bod(1,1),cs.j(1,2)],[bod(2,1),cs.j(2,2)],[bod(3,1),cs.j(3,2)],'Color','green','LineWidth',2)
        plot3(app.UIAxes,[bod(1,1),cs.k(1,2)],[bod(2,1),cs.k(2,2)],[bod(3,1),cs.k(3,2)],'Color','blue','LineWidth',2)
        
        plot3(app.UIAxes,[bod(1,2),cs.i(1,3)],[bod(2,2),cs.i(2,3)],[bod(3,2),cs.i(3,3)],'Color','red','LineWidth',2)
        plot3(app.UIAxes,[bod(1,2),cs.j(1,3)],[bod(2,2),cs.j(2,3)],[bod(3,2),cs.j(3,3)],'Color','green','LineWidth',2)
        plot3(app.UIAxes,[bod(1,2),cs.k(1,3)],[bod(2,2),cs.k(2,3)],[bod(3,2),cs.k(3,3)],'Color','blue','LineWidth',2)
        
        plot3(app.UIAxes,[bod(1,3),cs.i(1,5)],[bod(2,3),cs.i(2,5)],[bod(3,3),cs.i(3,5)],'Color','red','LineWidth',2)
        plot3(app.UIAxes,[bod(1,3),cs.j(1,5)],[bod(2,3),cs.j(2,5)],[bod(3,3),cs.j(3,5)],'Color','green','LineWidth',2)
        plot3(app.UIAxes,[bod(1,3),cs.k(1,5)],[bod(2,3),cs.k(2,5)],[bod(3,3),cs.k(3,5)],'Color','blue','LineWidth',2)
        
        plot3(app.UIAxes,[bod(1,4),cs.i(1,7)],[bod(2,4),cs.i(2,7)],[bod(3,4),cs.i(3,7)],'Color','red','LineWidth',2)
        plot3(app.UIAxes,[bod(1,4),cs.j(1,7)],[bod(2,4),cs.j(2,7)],[bod(3,4),cs.j(3,7)],'Color','green','LineWidth',2)
        plot3(app.UIAxes,[bod(1,4),cs.k(1,7)],[bod(2,4),cs.k(2,7)],[bod(3,4),cs.k(3,7)],'Color','blue','LineWidth',2)
    end

    if(app.drawXY == true)
        fill3(app.UIAxes,xy(1,:),xy(2,:),xy(3,:),'r','EdgeColor','none','FaceColor','red','FaceAlpha','0.2')
        plot3(app.UIAxes,xy(1,:),xy(2,:),xy(3,:),'Color','red','LineWidth',2)
        plot3(app.UIAxes,xy(1,:),xy(2,:),xy(3,:),'ok')
    end

    if(app.drawXZ == true)
        fill3(app.UIAxes,xz(1,:),xz(2,:),xz(3,:),'b','EdgeColor','none','FaceColor','blue','FaceAlpha','0.2')
        plot3(app.UIAxes,xz(1,:),xz(2,:),xz(3,:),'Color','blue','LineWidth',2)
    end
end