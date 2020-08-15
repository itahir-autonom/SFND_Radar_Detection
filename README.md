# SFND_Radar_Detection
Sensor Fusion Nanodegree Radar Project


This is Radar Target Generation and Detection Program for the Udacity Sensor Fusion Nanodegree.

The explaination of the Project as following

![alt text](https://github.com/itahir-autonom/SFND_Radar_Detection/blob/master/images/Process.png)


1. Target Initial Position and Velocity is taken Arbitrary
```
R_ini=110;   %Initial Position of the Target
Vo=60;       %Initial Velocity of the Target
```

2. Calculation of Bandwidth, Chrip Time and slope are

```
B=c/(2*R_res);  %Bandwidth of FMCW Radar
Tchirp=5.5*2*Rmax/c; %Chirp Time
slope=B/Tchirp; %slope
```

3. For all the time steps, Transmitted and Received Signal is calculated on 
![alt text](https://github.com/itahir-autonom/SFND_Radar_Detection/blob/master/images/Model%20Signal%20Propogation.png)

```
for i=1:length(t)         
    
    
    %For each time stamp update the Range of the Target for constant velocity. 
    r_t(i)=R_ini+Vo*t(i);     %position of the vehicle at time t
    td(i)=2*r_t(i)/c;         %Trip time of the signal\
    
    %For each time sample we need update the transmitted and
    %received signal. 
    Tx(i) = cos(2*pi*(fc*t(i)+slope*0.5*t(i)^2));
    Rx(i) = cos(2*pi*(fc*(t(i)-td(i))+slope*0.5*((t(i)-td(i))^2)));
    
    %Now by mixing the Transmit and Receive generate the beat signal
    %This is done by element wise matrix multiplication of Transmit and
    %Receiver Signal
    Mix(i) = Tx(i).*Rx(i);
    
end
```

4. Reshape the vector into Nd x Nr. Since it was already in the same dimensions, no need to do this

5. Apply FFT the mix signal we calculated in the loop version, which will give the signal from time domain to the frequency domain

![alt text](https://github.com/itahir-autonom/SFND_Radar_Detection/blob/master/images/FFT.png)

```
%run the FFT on the beat signal along the range bins dimension (Nr) and
%normalize.

signal_fft=fft(Mix,Nr)./Nr;


% Take the absolute value of FFT output
signal_fft=abs(signal_fft);


% Output of FFT is double sided signal, but we are interested in only one side of the spectrum.
% Hence we throw out half of the samples.
signal_fft=signal_fft(1:Nr/2);
```


![alt text](https://github.com/itahir-autonom/SFND_Radar_Detection/blob/master/images/1fft.jpg)


6. 2nd FFT was already implemented in the code, which is as under

![alt text](https://github.com/itahir-autonom/SFND_Radar_Detection/blob/master/images/2fft.jpg)

7. CFAR implementation is done by introducing a CFAR matrix of size RDM(2nd FFT) which is set to 0. Then using the training and guarding cells Tr,Td,Gr,Gd as given in the walkthrough video. Then Cut(slide) the cell across the complete matrix, then measures the noise through each iterations from the training cells.Then add offset to the threshold value calculated through noise. And if the cut level is more than threshold, set the value of CFAR =1.And this waz the target can be detected since it will above the threshold.

Steps.
7_1.choose the training as guard cells as mentioned in the walkthrough video 
```
Tr=10, Td=8, Gr=4, Gd=4 and offset =6
```
7_2.Introduce a matrix to save CFAR values of 0 and 1, which is equal to the size of RDM

7_3.Slide the cell under test across the complete matrix using a loop

7_4. Within the loop sum and average the noise level of all the training cells and adding offset value to it. This will be threshold.

7_5. Filter the cells in the matrix with respect to this threshold and if the signal is more than the threshold, assign the value to CFAR 1.

```
for i = Tr+Gr+1:Nr-(Tr+Gr)
    for j = Td+Gd+1:Nd-(Td+Gd)
        
        %Noise level Calculations
        noise_TG = db2pow(RDM(i-Tr-Gr:i+Tr+Gr,j-Td-Gd:j+Td+Gd));       %noise level of both training and guard cells
        noise_G = db2pow(RDM(i-Gr:i+Gr,j-Gd:j+Gd));                    %noise level of Guard cells
        noise_level = sum(noise_TG(:)) - sum(noise_G(:));              %noise level of Training cells
        
        %Threshold
        threshold = offset + pow2db(noise_level/(training_Cells/2));
        
        
        CUT = RDM(i,j);
        if(CUT <= threshold)
            CFAR(i,j) = 0;
        else
            CFAR(i, j) = 1;
        end        
    end
end
```
7_6. Since the initilized variable of CFAR was zero at the start, which means the cells at the edge didnt werenot considered under CUT and therefore were not processed. This gives the threshold block as

![alt text](https://github.com/itahir-autonom/SFND_Radar_Detection/blob/master/images/threshold%20block.jpg)

this was solved by taking the cells at edges to 0.

```
% The process above will generate a thresholded block, which is smaller 
%than the Range Doppler Map as the CUT cannot be located at the edges of
%matrix. Hence,few cells will not be thresholded. To keep the map size same
% set those values to 0. 

 
CFAR(1:Tr+Gr,:)=0;
CFAR(Nr-(Tr+Gr):Nr,:)=0;
CFAR(:,1:Td+Gd)=0;
CFAR(:,Nd-(Td+Gd):Nd)=0;


%display the CFAR output using the Surf function like we did for Range
%Doppler Response output.
figure,surf(doppler_axis,range_axis,CFAR);
colorbar;
```

And this resulted in the detection as



![alt text](https://github.com/itahir-autonom/SFND_Radar_Detection/blob/master/images/final.jpg)

