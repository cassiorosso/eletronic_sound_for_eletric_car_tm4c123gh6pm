% Esta mostra um vetor de tamanho "tamanho" com virgulas ap�s os n�meros
% Exemplo: temos o vetor vet = [2 3 1] com 3 espa�os, chamamos a fun��o
%          virgula(vet,3)
% A resposta ser�  2,
%                  3, 
%                  1,



function virgula(vetor,inicio,tamanho)
for i=inicio:1:tamanho
    formatspec='%8.0f,\n'; 
    fprintf(formatspec,double(vetor(i)))
end