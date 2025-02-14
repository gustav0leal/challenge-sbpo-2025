#!/bin/bash

# Verifica se os parâmetros de entrada foram fornecidos corretamente
if [[ $# -ne 2 ]]; then
    echo "Uso: $0 <pasta1> <pasta2>"
    exit 1
fi

# Atribui os parâmetros a variáveis
pasta1="$1"
pasta2="$2"

# Verifica se o script checker.py existe
if [[ ! -f ./checker.py ]]; then
    echo "Erro: O script 'checker.py' não foi encontrado."
    exit 1
fi

# Verifica se as pastas existem
if [[ ! -d "$pasta1" ]]; then
    echo "Erro: A pasta '$pasta1' não foi encontrada."
    exit 1
fi

if [[ ! -d "$pasta2" ]]; then
    echo "Erro: A pasta '$pasta2' não foi encontrada."
    exit 1
fi

# Percorre os arquivos da primeira pasta
for file1 in "$pasta1"/*; do
    # Obtém o nome do arquivo sem o caminho
    filename=$(basename "$file1")
    
    # Caminho correspondente na segunda pasta
    file2="$pasta2/$filename"
    
    # Verifica se o arquivo correspondente existe na segunda pasta
    if [[ -f "$file2" ]]; then
        echo "Executando checker.py com '$file1'"
        python3 checker.py "$file1" "$file2"
        # echo "---------------------------------"
    else
        echo "Aviso: Arquivo correspondente '$filename' não encontrado em '$pasta2'."
    fi
done
