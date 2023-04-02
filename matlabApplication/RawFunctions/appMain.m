function appMain(app)
    % cs for CoordinateSystem
    cs.i(:,1) = [50; 0; 0; 1]; 
    cs.j(:,1) = [0; 50; 0; 1];
    cs.k(:,1) = [0; 0; 50; 1];
    bod(:,1) = [0; 0; 0; 1];

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Výpočet robotického manipulátora
    %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Rotácia R1z
    cs.i(:,2) = (appRobRotate(app,'z','deg',90-app.phi1)*cs.i(:,1));
    cs.j(:,2) = (appRobRotate(app,'z','deg',90-app.phi1)*cs.j(:,1));
    cs.k(:,2) = (appRobRotate(app,'z','deg',90-app.phi1)*cs.k(:,1));
    % Translácia T2z
    bod(:,2) = (appRobRotate(app,'z','deg',90-app.phi1)*appRobTranslate(app,'z',app.l1)*bod(:,1));
    cs.i(:,3) = (appRobRotate(app,'z','deg',90-app.phi1)*appRobTranslate(app,'z',app.l1)*cs.i(:,1));
    cs.j(:,3) = (appRobRotate(app,'z','deg',90-app.phi1)*appRobTranslate(app,'z',app.l1)*cs.j(:,1));
    cs.k(:,3) = (appRobRotate(app,'z','deg',90-app.phi1)*appRobTranslate(app,'z',app.l1)*cs.k(:,1));
    % Rotácia R3y
    cs.i(:,4) = (appRobRotate(app,'z','deg',90-app.phi1)*appRobTranslate(app,'z',app.l1)*appRobRotate(app,'y','deg',app.phi2)*cs.i(:,1));
    cs.j(:,4) = (appRobRotate(app,'z','deg',90-app.phi1)*appRobTranslate(app,'z',app.l1)*appRobRotate(app,'y','deg',app.phi2)*cs.j(:,1));
    cs.k(:,4) = (appRobRotate(app,'z','deg',90-app.phi1)*appRobTranslate(app,'z',app.l1)*appRobRotate(app,'y','deg',app.phi2)*cs.k(:,1));
    % Translácia T4z
    bod(:,3) = (appRobRotate(app,'z','deg',90-app.phi1)*appRobTranslate(app,'z',app.l1)*appRobRotate(app,'y','deg',app.phi2)*appRobTranslate(app,'z',app.l2)*bod(:,1));
    cs.i(:,5) = (appRobRotate(app,'z','deg',90-app.phi1)*appRobTranslate(app,'z',app.l1)*appRobRotate(app,'y','deg',app.phi2)*appRobTranslate(app,'z',app.l2)*cs.i(:,1));
    cs.j(:,5) = (appRobRotate(app,'z','deg',90-app.phi1)*appRobTranslate(app,'z',app.l1)*appRobRotate(app,'y','deg',app.phi2)*appRobTranslate(app,'z',app.l2)*cs.j(:,1));
    cs.k(:,5) = (appRobRotate(app,'z','deg',90-app.phi1)*appRobTranslate(app,'z',app.l1)*appRobRotate(app,'y','deg',app.phi2)*appRobTranslate(app,'z',app.l2)*cs.k(:,1));
    % Rotácia R5y
    cs.i(:,6) = (appRobRotate(app,'z','deg',90-app.phi1)*appRobTranslate(app,'z',app.l1)*appRobRotate(app,'y','deg',app.phi2)*appRobTranslate(app,'z',app.l2)*appRobRotate(app,'y','deg',app.phi3)*cs.i(:,1));
    cs.j(:,6) = (appRobRotate(app,'z','deg',90-app.phi1)*appRobTranslate(app,'z',app.l1)*appRobRotate(app,'y','deg',app.phi2)*appRobTranslate(app,'z',app.l2)*appRobRotate(app,'y','deg',app.phi3)*cs.j(:,1));
    cs.k(:,6) = (appRobRotate(app,'z','deg',90-app.phi1)*appRobTranslate(app,'z',app.l1)*appRobRotate(app,'y','deg',app.phi2)*appRobTranslate(app,'z',app.l2)*appRobRotate(app,'y','deg',app.phi3)*cs.k(:,1));
    % Translácia T6z
    bod(:,4) = (appRobRotate(app,'z','deg',90-app.phi1)*appRobTranslate(app,'z',app.l1)*appRobRotate(app,'y','deg',app.phi2)*appRobTranslate(app,'z',app.l2)*appRobRotate(app,'y','deg',app.phi3)*appRobTranslate(app,'z',app.l3)*bod(:,1));
    cs.i(:,7) = (appRobRotate(app,'z','deg',90-app.phi1)*appRobTranslate(app,'z',app.l1)*appRobRotate(app,'y','deg',app.phi2)*appRobTranslate(app,'z',app.l2)*appRobRotate(app,'y','deg',app.phi3)*appRobTranslate(app,'z',app.l3)*cs.i(:,1));
    cs.j(:,7) = (appRobRotate(app,'z','deg',90-app.phi1)*appRobTranslate(app,'z',app.l1)*appRobRotate(app,'y','deg',app.phi2)*appRobTranslate(app,'z',app.l2)*appRobRotate(app,'y','deg',app.phi3)*appRobTranslate(app,'z',app.l3)*cs.j(:,1));
    cs.k(:,7) = (appRobRotate(app,'z','deg',90-app.phi1)*appRobTranslate(app,'z',app.l1)*appRobRotate(app,'y','deg',app.phi2)*appRobTranslate(app,'z',app.l2)*appRobRotate(app,'y','deg',app.phi3)*appRobTranslate(app,'z',app.l3)*cs.k(:,1));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Výpočet pracovného priestoru XY
    %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    xy_iter = 1;
    krokXY = 5;

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Algoritmus pre výpočet XY pracovného priestoru, prepočítaním 
    % vzdialenosti vektoru XY od počiatku [0,0,app.l1] so zaznamenávaním si 
    % len významných bodov, ktorých vzdialenosť od počiatku je väčšia ako v 
    % danom uhle zaznamenaná.
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    for(phi1_local = app.phi1_min:krokXY:app.phi1_max)
        xy(1:4,xy_iter) = 0;
        for(phi2_local = app.phi2_min:krokXY:0)
            for(phi3_local = app.phi3_min:krokXY:app.phi3_max)
                Ct(1:4) = (appRobRotate(app,'z','deg',90-phi1_local)*appRobTranslate(app,'z',app.l1)*appRobRotate(app,'y','deg',phi2_local)*appRobTranslate(app,'z',app.l2)*appRobRotate(app,'y','deg',phi3_local)*appRobTranslate(app,'z',app.l3)*bod(:,1));
                Ct(5) = sqrt(Ct(1)*Ct(1)+Ct(2)*Ct(2));
                if(xy(4,xy_iter) < Ct(5))
                    xy(:,xy_iter) = [Ct(1);Ct(2);app.l1;Ct(5)];
                end
            end
        end
        xy_iter = xy_iter + 1;
    end
    
    for(phi1_local = app.phi1_min:krokXY:app.phi1_max)
        xy(1:4,xy_iter) = 0;
        for(phi2_local = 0:krokXY:app.phi2_max)
            for(phi3_local = app.phi3_min:krokXY:app.phi3_max)
                Ct(1:4) = (appRobRotate(app,'z','deg',90-phi1_local)*appRobTranslate(app,'z',app.l1)*appRobRotate(app,'y','deg',phi2_local)*appRobTranslate(app,'z',app.l2)*appRobRotate(app,'y','deg',phi3_local)*appRobTranslate(app,'z',app.l3)*bod(:,1));
                Ct(5) = sqrt(Ct(1)*Ct(1)+Ct(2)*Ct(2));
                if(xy(4,xy_iter) < Ct(5))
                    xy(:,xy_iter) = [Ct(1);Ct(2);app.l1;Ct(5)];
                end
            end
        end
        xy_iter = xy_iter + 1;
    end
    xy(1:4,xy_iter)=xy(1:4,1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Výpočet pracovného priestoru ABC - YZ
    %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    xz_iter = 1;
    krokXZ = 5;

    phi1_local = 0;    
    phi2_local = app.phi2_min;
    for(phi3_local = app.phi3_min:krokXZ:app.phi3_max)
        Ct(1:4) = (appRobRotate(app,'z','deg',90-phi1_local)*appRobTranslate(app,'z',app.l1)*appRobRotate(app,'y','deg',phi2_local)*appRobTranslate(app,'z',app.l2)*appRobRotate(app,'y','deg',phi3_local)*appRobTranslate(app,'z',app.l3)*bod(:,1));
        Ct(5) = sqrt(Ct(1)*Ct(1)+Ct(2)*Ct(2));
        xz(:,xz_iter) = [Ct(1),Ct(2),Ct(3),Ct(5)];
        xz_iter = xz_iter + 1;
    end
    
    phi3_local = app.phi3_max;
    for(phi2_local = app.phi2_min:krokXZ:app.phi2_max)
        Ct(1:4) = (appRobRotate(app,'z','deg',90-phi1_local)*appRobTranslate(app,'z',app.l1)*appRobRotate(app,'y','deg',phi2_local)*appRobTranslate(app,'z',app.l2)*appRobRotate(app,'y','deg',phi3_local)*appRobTranslate(app,'z',app.l3)*bod(:,1));
        Ct(5) = sqrt(Ct(1)*Ct(1)+Ct(2)*Ct(2));
        xz(:,xz_iter) = [Ct(1),Ct(2),Ct(3),Ct(5)];
        xz_iter = xz_iter + 1;
    end
    
    phi2_local = app.phi2_max;
    for(phi3_local = app.phi3_max:-krokXZ:app.phi3_min)
        Ct(1:4) = (appRobRotate(app,'z','deg',90-phi1_local)*appRobTranslate(app,'z',app.l1)*appRobRotate(app,'y','deg',phi2_local)*appRobTranslate(app,'z',app.l2)*appRobRotate(app,'y','deg',phi3_local)*appRobTranslate(app,'z',app.l3)*bod(:,1));
        Ct(5) = sqrt(Ct(1)*Ct(1)+Ct(2)*Ct(2));
        xz(:,xz_iter) = [Ct(1),Ct(2),Ct(3),Ct(5)];
        xz_iter = xz_iter + 1;
    end
    
    phi3_local = app.phi3_min;
    for(phi2_local = app.phi2_max:-krokXZ:app.phi2_min)
        Ct(1:4) = (appRobRotate(app,'z','deg',90-phi1_local)*appRobTranslate(app,'z',app.l1)*appRobRotate(app,'y','deg',phi2_local)*appRobTranslate(app,'z',app.l2)*appRobRotate(app,'y','deg',phi3_local)*appRobTranslate(app,'z',app.l3)*bod(:,1));
        Ct(5) = sqrt(Ct(1)*Ct(1)+Ct(2)*Ct(2));
        xz(:,xz_iter) = [Ct(1),Ct(2),Ct(3),Ct(5)];
        xz_iter = xz_iter + 1;
    end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Vykreslenie údajov
    %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Nastavenie grafických prvkov pre jednotné zobrazenie

    cla(app.UIAxes);
    %     xlim([-400 400]);
    %     ylim([-400 400]);
    %     zlim([-100 600]);
    
    if(app.orientationLock == false)
        if(app.drawRM == true && app.drawXY == true && app.drawABC == true)
            title(app.UIAxes,"Robotic manipulator and his working areas");
            view(app.UIAxes,[130 30]);
        elseif((app.drawRM == true))
            title(app.UIAxes,"Robotic manipulator");
            view(app.UIAxes,[130 30]);
        elseif (app.drawXY == true)
            title(app.UIAxes,"XY working area");
             view(app.UIAxes,[0 90]);
        elseif (app.drawABC == true)
            title(app.UIAxes,"XZ working area");
             view(app.UIAxes,[90 0]);
        end
    end
    
    if(app.drawRM == true)
        % Vykreslenie ramien 0-A, A-B, B-C
        for (iterator = 2:1:length(bod))
            appRobPlot3(app,app.UIAxes,bod(:,iterator-1),bod(:,iterator))
            appRobSphere(app,app.UIAxes,7,bod(:,iterator))
        end
        
        % Vykreslenie súradnicových systémov v bodoch 0,A,B a v koncovom bode manipulátora C
        % Kvôli indexovaniu v matlabe sú dané indexy mätúce
        appRobPlotCS(app,app.UIAxes, bod, cs, 1, 2); % Vykresľujeme súradnice z bodu 0 (1) do Rz1*cs (2)
        appRobPlotCS(app,app.UIAxes, bod, cs, 2, 3); % Vykresľujeme súradnice z bodu A (2) do Rz1*Tz1*cs (3)
        appRobPlotCS(app,app.UIAxes, bod, cs, 3, 5); % Vykresľujeme súradnice z bodu B (3) do Rz1*Tz1*Ry2*Ty2*cs (5)
        appRobPlotCS(app,app.UIAxes, bod, cs, 4, 7); % Vykresľujeme súradnice z bodu C (4) do Rz1*Tz1*Ry2*Ty2*Ry3*Ty3*cs (3)
    end

    if(app.drawXY == true)
        % Vykreslenie pracovného priestoru XY
        fill3(app.UIAxes,xy(1,:),xy(2,:),xy(3,:),'r','EdgeColor','none','FaceColor','red','FaceAlpha','0.2')
        plot3(app.UIAxes,xy(1,:),xy(2,:),xy(3,:),'Color','red','LineWidth',2)
    end

    if(app.drawABC == true)
        % Vykreslenie pracovného priestoru ABC - XZ
        fill3(app.UIAxes,xz(1,:),xz(2,:),xz(3,:),'b','EdgeColor','none','FaceColor','blue','FaceAlpha','0.2')
        plot3(app.UIAxes,xz(1,:),xz(2,:),xz(3,:),'Color','blue','LineWidth',2)
    end
end