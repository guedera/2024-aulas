"use client"
import Image from 'next/image'

import notificaoces from "../../../../public/notificaoes.png"
import styles from './botao-notificacoes.module.css'

export default function BotaoDeNotificacoes({number, onClick}){

    return(
        <button className={styles.botao} onClick={onClick}>
            <Image
                className={styles.icone_notificacao}
                src={notificaoces}
                alt='notificacoes'
            ></Image>
            {number > 0 &&
                <div className={styles.numero}>{number}</div>
            }
        </button>
    )
}