% 
% Fun��o para realizar a Transformada de Fourier de um sinal
% e executa a plotagem no dom�nio frequ�ncia
%
% Autor: Wagner Rambo**
%
% **Engenheiro em Eletr�nica, Computadores e Telecomunica��es
%
%
% Maio de 2015
%
% Prot�tipo
%
% [X,freq] = my_fft(x,Fs)
% 
%
% ENTRADAS
% x � o sinal de entrada
% Fs � a frequ�ncia de amostragem do sinal
%
% SA�DAS
%
% X � o m�dulo do sinal no dom�nio frequ�ncia
% freq � o vetor de frequ�ncias
% 

function [X,freq] = my_fft(x,Fs)

N = length(x);                      % vari�vel N recebe o tamanho do vetor x
k = 0:N-1;                          % k � um vetor que vai de zero at� N menos 1
T = N/Fs;                           % Vetor de tempo N dividido pela frequ�ncia de amostragem
freq = k/T;
X = fftn(x)/N;                      % X recebe a FFT normalizada do vetor x sobre N
cutOff = ceil(N/2);                 % cutOff ajusta o eixo X
X = X(1:cutOff);
figure();
plot(freq(1:cutOff),abs(X));        % Plota a transformada de Fourier e o valor de X em m�dulo
title('Transformada R�pida de Fourier');
xlabel('Frequ�ncia (Hz)');
ylabel('Amplitude');


end