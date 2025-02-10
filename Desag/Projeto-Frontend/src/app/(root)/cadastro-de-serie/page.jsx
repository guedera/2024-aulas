"use client";
import React, { useState, useEffect } from 'react';
import { GET, POST } from '../_lib/actions';
import Dropdown from '../_components/dropdown';
import CampoTexto from '../_components/campo-texto';
import BotaoSalvar from '../_components/botao-salvar';
import Titulo from '../_components/titulo';
import Divisoria from '../_components/divisoria';
import styles from './page.module.css';


export default function Home() {
    const [idExercicio, setIdExercicio] = useState('');
    const [peso, setPeso] = useState('');
    const [numero, setNumero] = useState('');
    const [nome, setNome] = useState('');
    const [listaExercicios, setListaExercicios] = useState([]);
    const [mapaExercicios, setMapaExercicios] = useState({});
    const [erro, setErro] = useState();

    const [isActive, setIsActive] = useState(false);

    
    const construirNome = () => {
        const nomeExercicio = mapaExercicios[idExercicio] || '';
        return `${nomeExercicio} ${numero}x${peso} `;
    };

    useEffect(() => {
        setNome(construirNome());
    }, [idExercicio, peso, numero]);

    useEffect(() => {
        const camposPreenchidos = idExercicio && peso && numero;
        setIsActive(camposPreenchidos);
    }, [idExercicio, peso, numero]);

    // Busca os dados do endpoint '/exercicios' e configura o dropdown
    useEffect(() => {
        async function buscarDados() {
            try {
                const response = await GET('/exercicios');
                const mapaTemp = {};
                const dropdownList = response.map(exercicio => {
                    const id = exercicio.id;
                    const nome = exercicio.nome;
                    mapaTemp[id] = nome;
                    return {
                        value: id,
                        label: nome,
                    };
                    
                });
                console.log('Exercícios:', response);
                setMapaExercicios(mapaTemp);
                setListaExercicios(dropdownList);
            } catch (error) {
                setErro("Erro ao conectar ao servidor. Erro: " + error.message);
            }
        }
        buscarDados();
    }, []);


    const handleSave = async () => {
        const campos = [idExercicio, peso, numero];


        const camposPreenchidos = campos.every(campo => campo && campo.length > 0);

        setIsActive(camposPreenchidos);

        if (!camposPreenchidos) {
            alert('Cadastro não finalizado: preencha todos os campos.');
            return;
        };

        // Validação dos campos
        
        
        
            const serieData = {
                idDoExercicio: idExercicio,
                numeroDeVezes: numero,
                peso: peso,
            };

            try {
                await POST('/series', serieData);
                setNome('');
                setPeso('');
                setNumero('');
                setIdExercicio('');
                alert('Exercício salvo com sucesso!');
            } catch (error) {
                console.log('Erro ao salvar exercício:', error);
            }
        
    };

    return (
        <> 
            <main className={styles.container}>
                <Titulo>Cadastro de série</Titulo>
                <Dropdown name="Exercício" value={idExercicio} onChange={setIdExercicio} list={listaExercicios}/>
                <div className={styles.row}>
                    <div>
                        <CampoTexto name="Número de vezes" value={numero} onChange={setNumero} />
                    </div>
                    <div>
                        <CampoTexto name="Peso (kg)" value={peso} onChange={setPeso} />
                    </div>
                </div>
                <CampoTexto name="Nome" value={nome} onChange={setNome} readOnly />
                <Divisoria></Divisoria>
                <BotaoSalvar  onClick={handleSave} isActive={isActive} />
            </main>
        </>
    );
}
