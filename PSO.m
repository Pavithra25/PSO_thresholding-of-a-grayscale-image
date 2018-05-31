##%%PSO_thresholding of a grayscale image
clear;
close;
clc;

Dimensions = 1;
Xmin = 0;
Xmax = 256;
Vmin = 0;
Vmax = 256;

c1= 2;
c2= 2;
w=0.9;
img = imread('lena.jpg');
imshow(img);
s=imhist(img);
[m,n]=size(img);
N=n*m;
PopulationSize =20;
for i=1:256
        pix=s/N;
        summ=sum(pix);
end 
for l=1:256
    summean=sum(i*pix);
end
t=160
t2=256
for i=1
    wa=sum(pix(i:t))
end
for j=t+1
    wb=sum(pix(j:t2))
end
for q=1:t
    mean1=sum(q*pix)/w1;
end
 for e=t+1:t2
     mean2=sum(e*pix)/w2;
 end
 for g=1:256
 meanT=sum(g*pix(1:256))
 end
 for i=1:162
     var=sum((i-mean1).^2*pix(i)/w1);
 end
 for i=163:256
  var1=sum(i-mean2)^2*pix(i)/w1;
 end
 sum(pix)
MaxIterations = 256;
Positions_X  = Xmax*rand(PopulationSize,Dimensions);
Velocities_V = Vmax*rand(PopulationSize,Dimensions);
PBestPositions = Positions_X;
%----- Initial P_Best_Positions are the same as the  Positions_X-----------
for Particle=1:PopulationSize
    PBestFitnesses(Particle) = sum(Positions_X(Particle,:).^2);
end
[GBestFitness,GBestIndex] = min(PBestFitnesses);
GBestPosition = PBestPositions(GBestIndex,:);
%.....................intialization of population ,pbestfitness,........
%.....................pbestposition, gbestposition.......................
for Iteration = 1:MaxIterations
    w = 0.9 - 0.8 * Iteration/MaxIterations;
    for Particle = 1: PopulationSize
        Intertia              = w * Velocities_V(Particle,:);
        CognitionAcceleration = c1*rand*(-Positions_X(Particle,:)+ PBestPositions(Particle,:));
        SocialAcceleration    = c2*rand*(-Positions_X(Particle,:)+ GBestPosition);
        Velocities_V(Particle,:) = Intertia + CognitionAcceleration + SocialAcceleration;
        CurrentParticleVelocity = Velocities_V(Particle,:);
        %................Velocity Update..........................
        CurrentParticleVelocity(CurrentParticleVelocity > Vmax)=Vmax;
        CurrentParticleVelocity(CurrentParticleVelocity < Vmin)=Vmin;
        Velocities_V(Particle,:)= CurrentParticleVelocity;
        %...................Velocity Clamping......................
        CurrentPosition = Positions_X(Particle,:);
        NewPosition = CurrentPosition + Velocities_V(Particle,:);
        NewPosition(NewPosition > Xmax) = Xmax;
        NewPosition(NewPosition < Xmin) = Xmin;
        %..................Position Clamping....................
        Positions_X(Particle,:) = NewPosition;
        %...................Position Update........................
        Newfitness = sum(NewPosition.^2);
        if Newfitness < PBestFitnesses(Particle)
            PBestFitnesses(Particle) = Newfitness;
            PBestPositions(Particle,:) = NewPosition;
        end
        %.................Update the PBest.................................
        if Newfitness < GBestFitness
            GBestFitness  = Newfitness;
            GBestPosition = NewPosition;
        end  
        %.................Update the GBest.................................
    end
    fprintf('Iteration: %d  Bset Fitness: %e \n',Iteration,  GBestFitness)
    BestFitness(Iteration) = GBestFitness;
    end
plot(BestFitness);grid on;
GBestPosition
a=length(GBestPosition)
