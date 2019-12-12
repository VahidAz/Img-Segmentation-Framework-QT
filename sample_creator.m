img = zeros(200,200);

for i = 1:200
    for j = 1:200
        if ( j <= 100 )
            img(i,j) = 1;
        else
            img(i,j) = 2;
        end
    end
end
csvwrite( 'lbl_sample1.txt', img );
rgbImage = ind2rgb(img, lines(256));
imwrite(rgbImage, 'sample1.jpg', 'jpg');

for i = 1:200
    for j = 1:200
        if ( i <= 100 )
            img(i,j) = 1;
        else
            img(i,j) = 2;
        end
    end
end
csvwrite( 'lbl_sample2.txt', img );
rgbImage = ind2rgb(img, lines(256));
imwrite(rgbImage, 'sample2.jpg', 'jpg');

for i = 1:200
    for j = 1:200
        if ( i <= 100 && j <= 100 )
            img(i,j) = 1;
        end
        if ( i > 100 && j <= 100 )
            img(i,j) = 2;
        end
        if ( i <= 100 && j > 100 )
            img(i,j) = 3;
        end
        if ( i > 100 && j > 100 )
            img(i,j) = 4;
        end
    end
end
csvwrite( 'lbl_sample3.txt', img );
rgbImage = ind2rgb(img, lines(256));
imwrite(rgbImage, 'sample3.jpg', 'jpg');

for i = 1:200
    for j = 1:200
        if ( j <= 67 )
            img(i,j) = 1;
        else
            img(i,j) = 2;
        end
    end
end
csvwrite( 'lbl_sample4.txt', img );
rgbImage = ind2rgb(img, lines(256));
imwrite(rgbImage, 'sample4.jpg', 'jpg');
