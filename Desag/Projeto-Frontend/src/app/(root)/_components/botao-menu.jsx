import Image from 'next/image'
import hamburguer from "../../../../public/menuhamburguerpreto.png"
import styles from './botao-menu.module.css'

export default function BotaoDeMenu(props){
    return(
        <div>
            <button className={styles.botao} onClick={props.onClick}> 
                <Image
                    className={styles.icone_menu}
                    alt='menu hamburguer'
                    src={hamburguer}
                ></Image>
            </button>
        </div>
    )
}