% Esta mostra um vetor de tamanho "tamanho" com virgulas após os números
% Exemplo: temos o vetor vet = [2 3 1] com 3 espaços, chamamos a função
%          virgula(vet,3)
% A resposta será  2,
%                  3, 
%                  1,



function virgula(vetor,inicio,tamanho)
for i=inicio:1:tamanho
    formatspec='%8.0f,\n'; 
    fprintf(formatspec,double(vetor(i)))
end