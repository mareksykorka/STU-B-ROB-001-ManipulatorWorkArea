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
    cs.i(:,2) = (appRobRotate(app,'z','deg',app.phi1)*cs.i(:,1));
    cs.j(:,2) = (appRobRotate(app,'z','deg',app.phi1)*cs.j(:,1));
    cs.k(:,2) = (appRobRotate(app,'z','deg',app.phi1)*cs.k(:,1));
    % Translácia T2z
    bod(:,2) = (appRobRotate(app,'z','deg',app.phi1)*appRobTranslate(app,'z',app.l1)*bod(:,1));
    cs.i(:,3) = (appRobRotate(app,'z','deg',app.phi1)*appRobTranslate(app,'z',app.l1)*cs.i(:,1));
    cs.j(:,3) = (appRobRotate(app,'z','deg',app.phi1)*appRobTranslate(app,'z',app.l1)*cs.j(:,1));
    cs.k(:,3) = (appRobRotate(app,'z','deg',app.phi1)*appRobTranslate(app,'z',app.l1)*cs.k(:,1));
    % Rotácia R3y
    cs.i(:,4) = (appRobRotate(app,'z','deg',app.phi1)*appRobTranslate(app,'z',app.l1)*appRobRotate(app,'y','deg',app.phi2)*cs.i(:,1));
    cs.j(:,4) = (appRobRotate(app,'z','deg',app.phi1)*appRobTranslate(app,'z',app.l1)*appRobRotate(app,'y','deg',app.phi2)*cs.j(:,1));
    cs.k(:,4) = (appRobRotate(app,'z','deg',app.phi1)*appRobTranslate(app,'z',app.l1)*appRobRotate(app,'y','deg',app.phi2)*cs.k(:,1));
    % Translácia T4z
    bod(:,3) = (appRobRotate(app,'z','deg',app.phi1)*appRobTranslate(app,'z',app.l1)*appRobRotate(app,'y','deg',app.phi2)*appRobTranslate(app,'z',app.l2)*bod(:,1));
    cs.i(:,5) = (appRobRotate(app,'z','deg',app.phi1)*appRobTranslate(app,'z',app.l1)*appRobRotate(app,'y','deg',app.phi2)*appRobTranslate(app,'z',app.l2)*cs.i(:,1));
    cs.j(:,5) = (appRobRotate(app,'z','deg',app.phi1)*appRobTranslate(app,'z',app.l1)*appRobRotate(app,'y','deg',app.phi2)*appRobTranslate(app,'z',app.l2)*cs.j(:,1));
    cs.k(:,5) = (appRobRotate(app,'z','deg',app.phi1)*appRobTranslate(app,'z',app.l1)*appRobRotate(app,'y','deg',app.phi2)*appRobTranslate(app,'z',app.l2)*cs.k(:,1));
    % Rotácia R5y
    cs.i(:,6) = (appRobRotate(app,'z','deg',app.phi1)*appRobTranslate(app,'z',app.l1)*appRobRotate(app,'y','deg',app.phi2)*appRobTranslate(app,'z',app.l2)*appRobRotate(app,'y','deg',app.phi3)*cs.i(:,1));
    cs.j(:,6) = (appRobRotate(app,'z','deg',app.phi1)*appRobTranslate(app,'z',app.l1)*appRobRotate(app,'y','deg',app.phi2)*appRobTranslate(app,'z',app.l2)*appRobRotate(app,'y','deg',app.phi3)*cs.j(:,1));
    cs.k(:,6) = (appRobRotate(app,'z','deg',app.phi1)*appRobTranslate(app,'z',app.l1)*appRobRotate(app,'y','deg',app.phi2)*appRobTranslate(app,'z',app.l2)*appRobRotate(app,'y','deg',app.phi3)*cs.k(:,1));
    % Translácia T6z
    bod(:,4) = (appRobRotate(app,'z','deg',app.phi1)*appRobTranslate(app,'z',app.l1)*appRobRotate(app,'y','deg',app.phi2)*appRobTranslate(app,'z',app.l2)*appRobRotate(app,'y','deg',app.phi3)*appRobTranslate(app,'z',app.l3)*bod(:,1));
    cs.i(:,7) = (appRobRotate(app,'z','deg',app.phi1)*appRobTranslate(app,'z',app.l1)*appRobRotate(app,'y','deg',app.phi2)*appRobTranslate(app,'z',app.l2)*appRobRotate(app,'y','deg',app.phi3)*appRobTranslate(app,'z',app.l3)*cs.i(:,1));
    cs.j(:,7) = (appRobRotate(app,'z','deg',app.phi1)*appRobTranslate(app,'z',app.l1)*appRobRotate(app,'y','deg',app.phi2)*appRobTranslate(app,'z',app.l2)*appRobRotate(app,'y','deg',app.phi3)*appRobTranslate(app,'z',app.l3)*cs.j(:,1));
    cs.k(:,7) = (appRobRotate(app,'z','deg',app.phi1)*appRobTranslate(app,'z',app.l1)*appRobRotate(app,'y','deg',app.phi2)*appRobTranslate(app,'z',app.l2)*appRobRotate(app,'y','deg',app.phi3)*appRobTranslate(app,'z',app.l3)*cs.k(:,1));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Výpočet pracovného priestoru XY
    %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    xy_iter = 1;
    krokXY = 5;

    % Inicializacia xy pola
    xy = zeros(5,360/krokXY);
    for (i = 0:krokXY:360)
        xy(:,xy_iter) = [0;0;app.l1;0;i];
        xy_iter = xy_iter + 1;
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Algoritmus pre výpočet XY pracovného priestoru, prepočítaním 
    % vzdialenosti vektoru XY od počiatku [0,0,app.l1] so zaznamenávaním si len 
    % významných bodov, ktorých vzdialenosť od počiatku je väčšia ako v 
    % danom uhle zaznamenaná.
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    for(phi1_local = app.phi1_min:krokXY:app.phi1_max)
        for(phi2_local = app.phi2_min:krokXY:app.phi2_max)
            for(phi3_local = app.phi3_min:krokXY:app.phi3_max)
                Ct(1:4) = (appRobRotate(app,'z','deg',phi1_local)*appRobTranslate(app,'z',app.l1)*appRobRotate(app,'y','deg',phi2_local)*appRobTranslate(app,'z',app.l2)*appRobRotate(app,'y','deg',phi3_local)*appRobTranslate(app,'z',app.l3)*bod(:,1));
                Ct(5) = sqrt(Ct(1)*Ct(1)+Ct(2)*Ct(2));
    
                Ct(6) = atan2(Ct(2),Ct(1));
                Ct(6) = round(rad2deg(Ct(6)),0);
                if Ct(6) < 0
                    Ct(6) = Ct(6) + 360;
                end
    
                I = find(xy(5,:) == Ct(6));
                if(xy(4,I) < Ct(5))
                    xy(:,I) = [Ct(1);Ct(2);app.l1;Ct(5);Ct(6)];
                end 
            end
        end
    end
    % Doplnenie poľa dát, tak aby dáta boli "spojené", aby bolo
    % zobrazovanie krajšie.
    xy(1:4,end) = xy(1:4,1);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Úprava zozbieraných dát pre zlepšenie vizualizácie.
    %
    % Keďže výstupom algoritmu pre získanie pracovného priestoru XY sú
    % jednotlivé body v určitých unikátne definovaných uhloch pri grafickom
    % zobrazení daných uhlov vzniká problém, kde spojí body v uhle 90 a 95
    % priamou čiarou, teda sa javí akoby manipulátor mal väčší pracovný
    % priestor ako má reálne. Preto dopočítame body vnútornej kruižnice. 
    % Daná úprava taktiež nie je úplne perfektná.
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if(app.phi1_max+abs(app.phi1_min) >= 180)
        for (uhol = [app.phi1_min,app.phi1_max])
            uhol = mod(uhol, 360);
            % Nájdenie hraničného bodu
            I = find(xy(5,:) == uhol);
            val = xy(:,I-1:I+1);
    
            [M,Is] = min(val(4,:));
            smallest = val(:,Is);
            [M,Im] = max(val(4,:));
            biggest = val(:,Im);
            % Výpočet chýbajúceho bodu
            x = smallest(4)*cos(deg2rad(uhol));
            y = smallest(4)*sin(deg2rad(uhol));
    
            valIn = [x;y;app.l1;smallest(4);val(5,2)];
            % Dosadenie dát do poľa dát pre zobrazovanie
            if(xy(4,I-1) < xy(4,I))
                xy = [xy(:,1:I-1), valIn, xy(:,I:end)];
            else
                xy = [xy(:,1:I), valIn, xy(:,I+1:end)];
            end
        end
    end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Výpočet pracovného priestoru ABC - XZ
    %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    xz_iter = 1;
    krokXZ = 5;
    phi1_local = 0;
    
    phi2_local = app.phi2_min;
    for(phi3_local = app.phi3_min:krokXZ:app.phi3_max)
        Ct(1:4) = (appRobRotate(app,'z','deg',phi1_local)*appRobTranslate(app,'z',app.l1)*appRobRotate(app,'y','deg',phi2_local)*appRobTranslate(app,'z',app.l2)*appRobRotate(app,'y','deg',phi3_local)*appRobTranslate(app,'z',app.l3)*bod(:,1));
        Ct(5) = sqrt(Ct(1)*Ct(1)+Ct(2)*Ct(2));
        xz(:,xz_iter) = [Ct(1),Ct(2),Ct(3),Ct(5)];
        xz_iter = xz_iter + 1;
    end
    
    phi3_local = app.phi3_max;
    for(phi2_local = app.phi2_min:krokXZ:app.phi2_max)
        Ct(1:4) = (appRobRotate(app,'z','deg',phi1_local)*appRobTranslate(app,'z',app.l1)*appRobRotate(app,'y','deg',phi2_local)*appRobTranslate(app,'z',app.l2)*appRobRotate(app,'y','deg',phi3_local)*appRobTranslate(app,'z',app.l3)*bod(:,1));
        Ct(5) = sqrt(Ct(1)*Ct(1)+Ct(2)*Ct(2));
        xz(:,xz_iter) = [Ct(1),Ct(2),Ct(3),Ct(5)];
        xz_iter = xz_iter + 1;
    end
    
    phi2_local = app.phi2_max;
    for(phi3_local = app.phi3_max:-krokXZ:app.phi3_min)
        Ct(1:4) = (appRobRotate(app,'z','deg',phi1_local)*appRobTranslate(app,'z',app.l1)*appRobRotate(app,'y','deg',phi2_local)*appRobTranslate(app,'z',app.l2)*appRobRotate(app,'y','deg',phi3_local)*appRobTranslate(app,'z',app.l3)*bod(:,1));
        Ct(5) = sqrt(Ct(1)*Ct(1)+Ct(2)*Ct(2));
        xz(:,xz_iter) = [Ct(1),Ct(2),Ct(3),Ct(5)];
        xz_iter = xz_iter + 1;
    end
    
    phi3_local = app.phi3_min;
    for(phi2_local = app.phi2_max:-krokXZ:app.phi2_min)
        Ct(1:4) = (appRobRotate(app,'z','deg',phi1_local)*appRobTranslate(app,'z',app.l1)*appRobRotate(app,'y','deg',phi2_local)*appRobTranslate(app,'z',app.l2)*appRobRotate(app,'y','deg',phi3_local)*appRobTranslate(app,'z',app.l3)*bod(:,1));
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
    
    if((app.drawRM == true)||(app.drawRM == true && app.drawXY == true && app.drawXZ == true))
         view(app.UIAxes,[40 30]);
    elseif (app.drawXY == true)
         view(app.UIAxes,[0 90]);
    elseif (app.drawXZ == true)
         view(app.UIAxes,[90 0]);
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

    if(app.drawXZ == true)
        % Vykreslenie pracovného priestoru ABC - XZ
        fill3(app.UIAxes,xz(1,:),xz(2,:),xz(3,:),'b','EdgeColor','none','FaceColor','blue','FaceAlpha','0.2')
        plot3(app.UIAxes,xz(1,:),xz(2,:),xz(3,:),'Color','blue','LineWidth',2)
    end
end