% Lire l'image
image = imread('C:/Users/anton/Downloads/image2.png');
grayImage = rgb2gray(image);
binaryImage = imbinarize(grayImage);
binaryImage = ~binaryImage; % Inverser l'image binaire

% Appliquer une opération de fermeture pour relier les traits
se = strel('line', 5, 0); % Structuring element
closedImage = imclose(binaryImage, se); % Fermer les petits trous

% Distance Transform
distanceTransform = bwdist(~closedImage); % Calculer la distance transform
skeletonImage = distanceTransform > 0; % Binariser la distance transform

% seuil pour obtenir un squelette
skeletonImage = bwmorph(skeletonImage, 'thin', Inf);

% Chercher les segments
[L, num] = bwlabel(skeletonImage); % Étiqueter les composants connectés

% Obtenir les coordonnées des extrémités
endPoints = bwmorph(skeletonImage, 'endpoints'); % Détecter les points d'extrémité
[y, x] = find(endPoints); % Extraire les coordonnées des points d'extrémité
vPoints = [x, y]; % Combiner les coordonnées dans une matrice

% Filtrer les points pour exclure les courbures
filteredPoints = [];
for i = 1:size(vPoints, 1)
    x = vPoints(i, 1);
    y = vPoints(i, 2);
    
    % Vérifier la connectivité du point avec ses voisins
    neighbors = [
        x-1, y; x+1, y; x, y-1; x, y+1;   % Voisins directs
        x-1, y-1; x-1, y+1; x+1, y-1; x+1, y+1  % Voisins diagonaux
    ];
    
    % Compter le nombre de voisins connectés
    connCount = 0;
    for j = 1:size(neighbors, 1)
        if neighbors(j, 1) > 0 && neighbors(j, 1) <= size(skeletonImage, 2) && ...
           neighbors(j, 2) > 0 && neighbors(j, 2) <= size(skeletonImage, 1)
            connCount = connCount + skeletonImage(neighbors(j, 2), neighbors(j, 1));
        end
    end
    
    % Conserver uniquement les points avec un seul voisin (extrémité)
    if connCount == 1  
        filteredPoints = [filteredPoints; vPoints(i, :)]; % Conserver le point
    end
end

% Affichage de l'image originale
figure;
imshow(binaryImage);
hold on;

% Superposer les points extrêmes filtrés uniquement s'ils existent
if ~isempty(filteredPoints)
    plot(filteredPoints(:,1), filteredPoints(:,2), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
end
hold off;

% Afficher l'image squelettique
figure;
imshow(skeletonImage);
title('Squelette des lettres');